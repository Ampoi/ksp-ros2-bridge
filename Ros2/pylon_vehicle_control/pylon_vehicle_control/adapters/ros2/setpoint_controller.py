"""Lease-aware ROS2 adapter for the reusable 6DoF control law."""

import json
import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, TwistStamped
from pylon_interfaces.msg import (
    BodyWrenchCommand,
    ControlAuthorityCommand,
    ControlAuthorityState,
    ControlSetpoint,
    VesselLifecycle,
    WrenchFeedback,
)
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String

from ...application.lease import LeaseAction, LeaseCoordinator
from ...domain.control_law import (
    body_detumble_torque,
    body_wrench_for_setpoint,
    rate_guard_body_torque,
)
from ...domain.math3d import Quaternion, Vector3, add, linear_ramp_fraction, scale


class SetpointController(Node):
    """Convert coherent world-frame setpoints into owned, body-frame wrench commands."""

    def __init__(self) -> None:
        super().__init__("pylon_vehicle_setpoint_controller")
        self._declare_parameters()
        prefix = "/" + self._string("vessel_topic_prefix").strip("/")
        pose_topic = self._string("pose_topic") or f"{prefix}/ground_truth/pose"
        twist_topic = self._string("twist_topic") or f"{prefix}/ground_truth/twist"
        body_twist_topic = self._string("body_twist_topic") or f"{prefix}/ground_truth/twist_body"
        setpoint_topic = self._string("setpoint_topic") or f"{prefix}/control/setpoint"
        wrench_topic = self._string("wrench_command_topic") or f"{prefix}/control/wrench_command"
        authority_command_topic = self._string("authority_command_topic") or f"{prefix}/control/authority/command"
        authority_state_topic = self._string("authority_state_topic") or f"{prefix}/control/authority/state"
        lifecycle_topic = self._string("vessel_lifecycle_topic") or f"{prefix}/lifecycle"
        feedback_topic = self._string("wrench_feedback_topic") or f"{prefix}/control/wrench_feedback"
        status_topic = self._string("controller_status_topic") or f"{prefix}/control/controller_status"

        self.world_frame = self._string("world_frame")
        self.body_frame = self._string("body_frame")
        self.position_kp = self._positive("position_kp")
        self.velocity_kd = self._positive("velocity_kd")
        self.attitude_kp = self._positive("attitude_kp")
        self.angular_kd = self._positive("angular_kd")
        self.max_force = self._positive("max_force")
        self.max_torque = self._positive("max_torque")
        self.attitude_hold_max_torque = self._positive("attitude_hold_max_torque")
        self.attitude_hold_rate_limit = math.radians(
            self._positive("attitude_hold_rate_limit_deg_s")
        )
        self.detumble_kd = self._positive("detumble_kd")
        self.detumble_max_torque = self._positive("detumble_max_torque")
        self.command_ramp_sec = self._positive("command_ramp_sec")
        self.command_timeout_sec = self._positive("command_timeout_sec")
        self.lease_duration_sec = self._positive("lease_duration_sec")
        self.priority = int(self.get_parameter("control_priority").value)
        self.suppress_sas = bool(self.get_parameter("suppress_sas").value)
        self.state_timeout = Duration(seconds=self._positive("state_timeout_sec"))
        self.setpoint_timeout = Duration(seconds=self._positive("setpoint_timeout_sec"))
        self.extrapolate_setpoint = bool(self.get_parameter("extrapolate_setpoint").value)
        self.safety_cooldown = Duration(
            seconds=self._positive("control_safety_cooldown_sec")
        )

        self.pose: Optional[PoseStamped] = None
        self.twist: Optional[TwistStamped] = None
        self.body_twist: Optional[TwistStamped] = None
        self.setpoint: Optional[ControlSetpoint] = None
        self.pose_received: Optional[Time] = None
        self.twist_received: Optional[Time] = None
        self.body_twist_received: Optional[Time] = None
        self.setpoint_received: Optional[Time] = None
        self.active_mode = ControlSetpoint.MODE_IDLE
        self.mode_started: Optional[Time] = None
        self.safety_cooldown_until: Optional[Time] = None
        self.control_interrupted = False
        self.last_status = ""
        self.sequence = 0
        self.lease = LeaseCoordinator(
            self._string("controller_id"), self._positive("lease_renew_period_sec")
        )

        command_qos = QoSProfile(depth=10)
        command_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.wrench_publisher = self.create_publisher(BodyWrenchCommand, wrench_topic, command_qos)
        self.authority_publisher = self.create_publisher(
            ControlAuthorityCommand, authority_command_topic, command_qos
        )
        self.status_publisher = self.create_publisher(String, status_topic, status_qos)
        self.create_subscription(PoseStamped, pose_topic, self.receive_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, twist_topic, self.receive_twist, qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, body_twist_topic, self.receive_body_twist, qos_profile_sensor_data
        )
        self.create_subscription(ControlSetpoint, setpoint_topic, self.receive_setpoint, 10)
        self.create_subscription(
            ControlAuthorityState, authority_state_topic, self.receive_authority, state_qos
        )
        self.create_subscription(
            VesselLifecycle, lifecycle_topic, self.receive_lifecycle, state_qos
        )
        self.create_subscription(
            WrenchFeedback, feedback_topic, self.receive_wrench_feedback, command_qos
        )
        self.create_timer(1.0 / self._positive("control_rate_hz"), self.control)
        self.get_logger().info(
            f"controller={self.lease.controller_id} setpoint={setpoint_topic} "
            f"wrench={wrench_topic} lifecycle={lifecycle_topic}"
        )

    def _declare_parameters(self) -> None:
        values = {
            "controller_id": "setpoint_controller",
            "vessel_topic_prefix": "/ksp_vessel",
            "pose_topic": "",
            "twist_topic": "",
            "body_twist_topic": "",
            "setpoint_topic": "",
            "wrench_command_topic": "",
            "authority_command_topic": "",
            "authority_state_topic": "",
            "vessel_lifecycle_topic": "",
            "wrench_feedback_topic": "",
            "controller_status_topic": "",
            "world_frame": "pylon_ground_truth_enu",
            "body_frame": "base_link",
            "control_priority": 100,
            "lease_duration_sec": 2.0,
            "lease_renew_period_sec": 0.5,
            "suppress_sas": True,
            "command_timeout_sec": 0.25,
            "position_kp": 500.0,
            "velocity_kd": 2000.0,
            "attitude_kp": 800.0,
            "angular_kd": 4000.0,
            "max_force": 2000.0,
            "max_torque": 250.0,
            "attitude_hold_max_torque": 150.0,
            "attitude_hold_rate_limit_deg_s": 2.5,
            "detumble_kd": 1000.0,
            "detumble_max_torque": 500.0,
            "command_ramp_sec": 3.0,
            "state_timeout_sec": 0.5,
            "setpoint_timeout_sec": 0.5,
            "extrapolate_setpoint": False,
            "control_safety_cooldown_sec": 0.75,
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

    def receive_lifecycle(self, message: VesselLifecycle) -> None:
        active = message.state in (
            VesselLifecycle.STATE_ACTIVE,
            VesselLifecycle.STATE_CHANGED,
        )
        previous_vessel = self.lease.vessel_id
        if self.lease.observe_vessel(message.vessel_id, active, message.generation):
            if previous_vessel:
                self.control_interrupted = True
            self.pose = self.twist = self.body_twist = self.setpoint = None
            self.pose_received = self.twist_received = self.body_twist_received = self.setpoint_received = None

    def receive_authority(self, message: ControlAuthorityState) -> None:
        lost = self.lease.observe_authority(
            message.vessel_id,
            message.controller_id,
            message.lease_id,
            message.state == ControlAuthorityState.STATE_OWNED,
        )

        if lost:
            self.control_interrupted = True
            self.setpoint = None

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

    def receive_body_twist(self, message: TwistStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.body_frame:
            return
        self.body_twist = message
        self.body_twist_received = self.get_clock().now()

    def receive_setpoint(self, message: ControlSetpoint) -> None:
        if message.header.frame_id and message.header.frame_id != self.world_frame:
            self.get_logger().warning(
                f"Ignoring setpoint in {message.header.frame_id}; expected {self.world_frame}",
                throttle_duration_sec=5.0,
            )
            return
        if self.control_interrupted:
            if message.mode != ControlSetpoint.MODE_IDLE:
                return
            self.control_interrupted = False
        now = self.get_clock().now()
        if message.mode != self.active_mode:
            self.active_mode = message.mode
            self.mode_started = now
        self.setpoint = message
        self.setpoint_received = now

    def receive_wrench_feedback(self, message: WrenchFeedback) -> None:
        if (
            message.controller_id == self.lease.controller_id
            and "continuous_actuation_limit" in message.reason.split(",")
            and self.safety_cooldown_until is None
        ):
            self.safety_cooldown_until = self.get_clock().now() + self.safety_cooldown

    def control(self) -> None:
        now = self.get_clock().now()
        if self.control_interrupted:
            self._publish_status("control_interrupted")
            return
        if not self._setpoint_requests_control(now):
            if self.lease.owned:
                self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
                self._release_authority(now)
            if self.setpoint is None:
                state = "waiting_for_setpoint"
            elif self.setpoint.mode == ControlSetpoint.MODE_IDLE:
                state = "idle"
            else:
                state = "failsafe_idle"
            self._publish_status(state)
            return
        if not self._inputs_are_fresh(now):
            if self.lease.owned:
                self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
                self._release_authority(now)
            self.control_interrupted = True
            self.setpoint = None
            self.active_mode = ControlSetpoint.MODE_IDLE
            self.mode_started = None
            self._publish_status("control_interrupted")
            return
        if self.safety_cooldown_until is not None:
            self._maintain_authority(now)
            if now < self.safety_cooldown_until:
                if self.lease.owned:
                    self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
                self._publish_status("safety_cooldown")
                return
            self.safety_cooldown_until = None
            self.mode_started = now
        self._maintain_authority(now)
        if not self.lease.owned:
            self._publish_status(
                "waiting_for_control_authority",
                vessel_id=self.lease.vessel_id,
            )
            return

        assert (
            self.pose is not None
            and self.twist is not None
            and self.body_twist is not None
            and self.setpoint is not None
        )
        mode = self.setpoint.mode
        if mode == ControlSetpoint.MODE_IDLE:
            self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            self._publish_status("idle")
            return

        orientation = self._orientation()
        angular_velocity_body = self._angular_velocity_body()
        if mode == ControlSetpoint.MODE_DETUMBLE:
            force = (0.0, 0.0, 0.0)
            torque = body_detumble_torque(
                angular_velocity_body, self.detumble_kd, self.detumble_max_torque
            )
            state = "detumbling"
        elif mode in (ControlSetpoint.MODE_ATTITUDE_HOLD, ControlSetpoint.MODE_SIX_DOF):
            desired = self.setpoint
            desired_position = (desired.position.x, desired.position.y, desired.position.z)
            if self.extrapolate_setpoint:
                dt = (Time.from_msg(self.pose.header.stamp) - Time.from_msg(desired.header.stamp)).nanoseconds * 1.0e-9
                if abs(dt) > self.setpoint_timeout.nanoseconds * 1.0e-9:
                    self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
                    self._release_authority(now)
                    self._publish_status("setpoint_timestamp_mismatch")
                    return
                desired_position = add(desired_position, scale(
                    (desired.linear_velocity.x, desired.linear_velocity.y, desired.linear_velocity.z), dt))
            force, torque = body_wrench_for_setpoint(
                self._position(), orientation, self._linear_velocity(), angular_velocity_body,
                desired_position,
                (desired.orientation.x, desired.orientation.y, desired.orientation.z, desired.orientation.w),
                (desired.linear_velocity.x, desired.linear_velocity.y, desired.linear_velocity.z),
                (desired.angular_velocity.x, desired.angular_velocity.y, desired.angular_velocity.z),
                self.position_kp, self.velocity_kd, self.attitude_kp, self.angular_kd,
                self.max_force,
                self.max_torque if mode == ControlSetpoint.MODE_SIX_DOF else min(
                    self.max_torque, self.attitude_hold_max_torque
                ),
                mode == ControlSetpoint.MODE_SIX_DOF,
                self.attitude_hold_rate_limit,
            )
            torque = rate_guard_body_torque(
                torque,
                angular_velocity_body,
                self.attitude_hold_rate_limit,
                self.angular_kd,
                self.max_torque if mode == ControlSetpoint.MODE_SIX_DOF else min(
                    self.max_torque, self.attitude_hold_max_torque
                ),
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

    def _maintain_authority(self, now: Time) -> None:
        action = self.lease.due_action(now.nanoseconds * 1.0e-9)
        if action is not None:
            self._publish_authority(action, now)

    def _release_authority(self, now: Time) -> None:
        action = self.lease.release_action()
        if action is not None:
            self._publish_authority(action, now)

    def _publish_authority(self, action: LeaseAction, now: Time) -> None:
        self.sequence += 1
        message = ControlAuthorityCommand()
        message.header.stamp = now.to_msg()
        message.action = {
            "acquire": ControlAuthorityCommand.ACTION_ACQUIRE,
            "renew": ControlAuthorityCommand.ACTION_RENEW,
            "release": ControlAuthorityCommand.ACTION_RELEASE,
        }[action.action]
        message.vessel_id = action.vessel_id
        message.controller_id = action.controller_id
        message.lease_id = action.lease_id
        message.priority = self.priority
        message.lease_duration_sec = self.lease_duration_sec
        message.suppress_sas = self.suppress_sas
        message.sequence = self.sequence
        self.authority_publisher.publish(message)

    def _inputs_are_fresh(self, now: Time) -> bool:
        return bool(
            self.pose is not None and self.twist is not None
            and self.body_twist is not None and self.setpoint is not None
            and self.pose_received is not None and now - self.pose_received <= self.state_timeout
            and self.twist_received is not None and now - self.twist_received <= self.state_timeout
            and self.body_twist_received is not None
            and now - self.body_twist_received <= self.state_timeout
            and self.setpoint_received is not None and now - self.setpoint_received <= self.setpoint_timeout
        )

    def _setpoint_requests_control(self, now: Time) -> bool:
        return bool(
            self.setpoint is not None
            and self.setpoint_received is not None
            and now - self.setpoint_received <= self.setpoint_timeout
            and self.setpoint.mode in (
                ControlSetpoint.MODE_DETUMBLE,
                ControlSetpoint.MODE_ATTITUDE_HOLD,
                ControlSetpoint.MODE_SIX_DOF,
            )
        )

    def _position(self) -> Vector3:
        assert self.pose is not None
        value = self.pose.pose.position
        return value.x, value.y, value.z

    def _orientation(self) -> Quaternion:
        assert self.pose is not None
        value = self.pose.pose.orientation
        return value.x, value.y, value.z, value.w

    def _linear_velocity(self) -> Vector3:
        assert self.twist is not None
        value = self.twist.twist.linear
        return value.x, value.y, value.z

    def _angular_velocity_body(self) -> Vector3:
        assert self.body_twist is not None
        value = self.body_twist.twist.angular
        return value.x, value.y, value.z

    def _publish_wrench(self, now: Time, force: Vector3, torque: Vector3) -> None:
        self.sequence += 1
        message = BodyWrenchCommand()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self.body_frame
        message.vessel_id = self.lease.vessel_id
        message.controller_id = self.lease.controller_id
        message.lease_id = self.lease.lease_id
        message.sequence = self.sequence
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = force
        message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z = torque
        message.timeout_sec = self.command_timeout_sec
        self.wrench_publisher.publish(message)

    def _publish_status(self, state: str, **extra: object) -> None:
        serialized = json.dumps({"state": state, **extra}, sort_keys=True)
        if serialized == self.last_status:
            return
        self.last_status = serialized
        self.status_publisher.publish(String(data=serialized))

    def destroy_node(self) -> bool:
        if rclpy.ok():
            self._release_authority(self.get_clock().now())
        return super().destroy_node()


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
