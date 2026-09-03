"""Generic 6DoF setpoint controller for the debris-orbit demo."""

from __future__ import annotations

import json
import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from ksp_ros2_interfaces.msg import ControlSetpoint
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String

from .control import body_detumble_torque, body_wrench_for_setpoint
from .core import Quaternion, Vector3, linear_ramp_fraction, scale, vessel_topics


class SetpointController(Node):
    """Convert coherent pose/velocity setpoints into the bridge's body Wrench API."""

    def __init__(self) -> None:
        super().__init__("debris_orbit_controller")
        self._declare_parameters()
        platform_id = self._string("platform_id")
        prefix = self._string("vessel_topic_prefix").rstrip("/")
        defaults = vessel_topics(prefix, "unused", "unused", platform_id)
        pose_topic = self._string("pose_topic") or defaults.ground_truth_pose
        twist_topic = self._string("twist_topic") or defaults.ground_truth_twist
        setpoint_topic = self._string("setpoint_topic") or defaults.control_setpoint
        wrench_topic = self._string("body_wrench_topic") or defaults.body_wrench
        status_topic = self._string("controller_status_topic") or defaults.controller_status

        self.world_frame = self._string("world_frame")
        self.body_frame = self._string("body_frame")
        self.position_kp = self._positive("position_kp")
        self.velocity_kd = self._positive("velocity_kd")
        self.attitude_kp = self._positive("attitude_kp")
        self.angular_kd = self._positive("angular_kd")
        self.max_force = self._positive("max_force")
        self.max_torque = self._positive("max_torque")
        self.detumble_kd = self._positive("detumble_kd")
        self.detumble_max_torque = self._positive("detumble_max_torque")
        self.command_ramp_sec = self._positive("command_ramp_sec")
        self.state_timeout = Duration(seconds=self._positive("state_timeout_sec"))
        self.setpoint_timeout = Duration(seconds=self._positive("setpoint_timeout_sec"))

        self.pose: Optional[PoseStamped] = None
        self.twist: Optional[TwistStamped] = None
        self.setpoint: Optional[ControlSetpoint] = None
        self.pose_received: Optional[Time] = None
        self.twist_received: Optional[Time] = None
        self.setpoint_received: Optional[Time] = None
        self.active_mode = ControlSetpoint.MODE_IDLE
        self.mode_started: Optional[Time] = None
        self.last_status = ""

        self.wrench_publisher = self.create_publisher(WrenchStamped, wrench_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.create_subscription(PoseStamped, pose_topic, self.receive_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, twist_topic, self.receive_twist, qos_profile_sensor_data)
        self.create_subscription(ControlSetpoint, setpoint_topic, self.receive_setpoint, 10)
        self.create_timer(1.0 / self._positive("control_rate_hz"), self.control)
        self.get_logger().info(
            f"setpoint={setpoint_topic} wrench={wrench_topic} pose={pose_topic} twist={twist_topic}"
        )

    def _declare_parameters(self) -> None:
        values = {
            "platform_id": "demo_vehicle",
            "vessel_topic_prefix": "/ksp_vessel",
            "pose_topic": "",
            "twist_topic": "",
            "setpoint_topic": "",
            "body_wrench_topic": "",
            "controller_status_topic": "",
            "world_frame": "ground_truth_enu",
            "body_frame": "base_link",
            "position_kp": 120.0,
            "velocity_kd": 350.0,
            "attitude_kp": 800.0,
            "angular_kd": 300.0,
            "max_force": 5000.0,
            "max_torque": 300.0,
            "detumble_kd": 50.0,
            "detumble_max_torque": 100.0,
            "command_ramp_sec": 2.0,
            "state_timeout_sec": 0.5,
            "setpoint_timeout_sec": 0.5,
            "control_rate_hz": 20.0,
        }
        for name, default in values.items():
            self.declare_parameter(name, default)

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"parameter {name} must be finite and greater than zero")
        return value

    def receive_pose(self, message: PoseStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.world_frame:
            return
        self.pose = message
        self.pose_received = self.get_clock().now()

    def receive_twist(self, message: TwistStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.world_frame:
            return
        self.twist = message
        self.twist_received = self.get_clock().now()

    def receive_setpoint(self, message: ControlSetpoint) -> None:
        if message.header.frame_id and message.header.frame_id != self.world_frame:
            self.get_logger().warning(
                f"Ignoring setpoint in {message.header.frame_id}; expected {self.world_frame}",
                throttle_duration_sec=5.0,
            )
            return
        now = self.get_clock().now()
        if message.mode != self.active_mode:
            self.active_mode = message.mode
            self.mode_started = now
        self.setpoint = message
        self.setpoint_received = now

    def control(self) -> None:
        now = self.get_clock().now()
        if not self._inputs_are_fresh(now):
            self.active_mode = ControlSetpoint.MODE_IDLE
            self.mode_started = None
            self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            self._publish_status("failsafe_idle")
            return
        assert self.pose is not None and self.twist is not None and self.setpoint is not None
        mode = self.setpoint.mode
        if mode == ControlSetpoint.MODE_IDLE:
            self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            self._publish_status("idle")
            return

        orientation = self._orientation()
        angular_velocity = self._angular_velocity()
        if mode == ControlSetpoint.MODE_DETUMBLE:
            force = (0.0, 0.0, 0.0)
            torque = body_detumble_torque(
                orientation, angular_velocity, self.detumble_kd, self.detumble_max_torque
            )
            state = "detumbling"
        elif mode in (ControlSetpoint.MODE_ATTITUDE_HOLD, ControlSetpoint.MODE_SIX_DOF):
            desired = self.setpoint
            force, torque = body_wrench_for_setpoint(
                self._position(), orientation, self._linear_velocity(), angular_velocity,
                (desired.position.x, desired.position.y, desired.position.z),
                (desired.orientation.x, desired.orientation.y, desired.orientation.z, desired.orientation.w),
                (desired.linear_velocity.x, desired.linear_velocity.y, desired.linear_velocity.z),
                (desired.angular_velocity.x, desired.angular_velocity.y, desired.angular_velocity.z),
                self.position_kp, self.velocity_kd, self.attitude_kp, self.angular_kd,
                self.max_force, self.max_torque,
                mode == ControlSetpoint.MODE_SIX_DOF,
            )
            state = "six_dof" if mode == ControlSetpoint.MODE_SIX_DOF else "attitude_hold"
        else:
            self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            self._publish_status("invalid_mode", mode=int(mode))
            return

        ramp = linear_ramp_fraction(
            0.0 if self.mode_started is None else (now - self.mode_started).nanoseconds * 1.0e-9,
            self.command_ramp_sec,
        )
        self._publish_wrench(now, scale(force, ramp), scale(torque, ramp))
        self._publish_status(state, ramp=round(ramp, 2))

    def _inputs_are_fresh(self, now: Time) -> bool:
        return bool(
            self.pose is not None and self.twist is not None and self.setpoint is not None
            and self.pose_received is not None and now - self.pose_received <= self.state_timeout
            and self.twist_received is not None and now - self.twist_received <= self.state_timeout
            and self.setpoint_received is not None and now - self.setpoint_received <= self.setpoint_timeout
        )

    def _position(self) -> Vector3:
        assert self.pose is not None
        p = self.pose.pose.position
        return (p.x, p.y, p.z)

    def _orientation(self) -> Quaternion:
        assert self.pose is not None
        q = self.pose.pose.orientation
        return (q.x, q.y, q.z, q.w)

    def _linear_velocity(self) -> Vector3:
        assert self.twist is not None
        v = self.twist.twist.linear
        return (v.x, v.y, v.z)

    def _angular_velocity(self) -> Vector3:
        assert self.twist is not None
        v = self.twist.twist.angular
        return (v.x, v.y, v.z)

    def _publish_wrench(self, now: Time, force: Vector3, torque: Vector3) -> None:
        message = WrenchStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self.body_frame
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = force
        message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z = torque
        self.wrench_publisher.publish(message)

    def _publish_status(self, state: str, **extra: object) -> None:
        serialized = json.dumps({"state": state, **extra}, sort_keys=True)
        if serialized == self.last_status:
            return
        self.last_status = serialized
        message = String()
        message.data = serialized
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SetpointController()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
