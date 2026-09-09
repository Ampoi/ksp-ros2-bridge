"""Lease-aware ROS2 adapter from Nav2 velocity commands to vessel control."""

import math
from typing import Optional

from geometry_msgs.msg import Twist
from pylon_interfaces.msg import (
    BodyWrenchCommand,
    ControlAuthorityCommand,
    ControlAuthorityState,
    VesselLifecycle,
    WrenchFeedback,
)
from pylon_vehicle_control.application.lease import LeaseAction, LeaseCoordinator
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time

from .math_utils import planar_wrench, rotate_planar
from .session import SessionGuard


class PlanarWrenchController(Node):
    """Track Nav2 velocity commands while exclusively owning one vessel lease."""

    def __init__(self) -> None:
        """Create controller parameters, ROS interfaces, and the control timer."""
        super().__init__("pylon_planar_wrench_controller")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/pylon/lidar_slam/odom")
        self.declare_parameter(
            "wrench_command_topic", "/ksp_vessel/control/wrench_command"
        )
        self.declare_parameter(
            "authority_command_topic", "/ksp_vessel/control/authority/command"
        )
        self.declare_parameter(
            "authority_state_topic", "/ksp_vessel/control/authority/state"
        )
        self.declare_parameter(
            "wrench_feedback_topic", "/ksp_vessel/control/wrench_feedback"
        )
        self.declare_parameter("vessel_lifecycle_topic", "/ksp_vessel/lifecycle")
        self.declare_parameter("controller_id", "nav2_planar_controller")
        self.declare_parameter("control_priority", 80)
        self.declare_parameter("lease_duration_sec", 2.0)
        self.declare_parameter("lease_renew_period_sec", 0.5)
        self.declare_parameter("suppress_sas", True)
        self.declare_parameter("wrench_timeout_sec", 0.25)
        self.declare_parameter("odom_base_frame", "pylon_slam_base_link")
        self.declare_parameter("wrench_frame", "base_link")
        self.declare_parameter("nav_to_body_yaw", 0.0)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("linear_gain", 1000.0)
        self.declare_parameter("angular_gain", 1000.0)
        self.declare_parameter("max_planar_force", 10000.0)
        self.declare_parameter("max_yaw_torque", 10000.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("odom_timeout", 0.25)
        self.declare_parameter("brake_duration", 0.5)
        self.declare_parameter("control_safety_cooldown_sec", 0.75)

        self.odom_base_frame = str(self.get_parameter("odom_base_frame").value)
        self.wrench_frame = str(self.get_parameter("wrench_frame").value)
        self.nav_to_body_yaw = float(self.get_parameter("nav_to_body_yaw").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.max_force = float(self.get_parameter("max_planar_force").value)
        self.max_torque = float(self.get_parameter("max_yaw_torque").value)
        self.command_timeout = Duration(
            seconds=float(self.get_parameter("command_timeout").value)
        )
        self.odom_timeout = Duration(
            seconds=float(self.get_parameter("odom_timeout").value)
        )
        self.brake_duration = Duration(
            seconds=float(self.get_parameter("brake_duration").value)
        )
        self.release_timeout = Duration(
            nanoseconds=self.command_timeout.nanoseconds + self.brake_duration.nanoseconds
        )
        self.safety_cooldown = Duration(
            seconds=float(self.get_parameter("control_safety_cooldown_sec").value)
        )
        self.control_priority = int(self.get_parameter("control_priority").value)
        self.lease_duration_sec = float(
            self.get_parameter("lease_duration_sec").value
        )
        self.suppress_sas = bool(self.get_parameter("suppress_sas").value)
        self.wrench_timeout_sec = float(
            self.get_parameter("wrench_timeout_sec").value
        )
        self.session = SessionGuard()
        self.sequence = 0
        self.lease = LeaseCoordinator(
            str(self.get_parameter("controller_id").value),
            float(self.get_parameter("lease_renew_period_sec").value),
        )
        rate = max(1.0, float(self.get_parameter("control_rate").value))

        self.target = (0.0, 0.0, 0.0)
        self.measured = (0.0, 0.0, 0.0)
        self.last_command: Optional[Time] = None
        self.last_odometry: Optional[Time] = None
        self.safety_cooldown_until: Optional[Time] = None

        command_qos = QoSProfile(depth=10)
        command_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            BodyWrenchCommand,
            str(self.get_parameter("wrench_command_topic").value),
            command_qos,
        )
        self.authority_publisher = self.create_publisher(
            ControlAuthorityCommand,
            str(self.get_parameter("authority_command_topic").value),
            command_qos,
        )
        self.command_subscription = self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.receive_command,
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.receive_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ControlAuthorityState,
            str(self.get_parameter("authority_state_topic").value),
            self.receive_authority,
            state_qos,
        )
        self.create_subscription(
            VesselLifecycle,
            str(self.get_parameter("vessel_lifecycle_topic").value),
            self.receive_lifecycle,
            state_qos,
        )
        self.create_subscription(
            WrenchFeedback,
            str(self.get_parameter("wrench_feedback_topic").value),
            self.receive_wrench_feedback,
            command_qos,
        )
        self.timer = self.create_timer(1.0 / rate, self.control)

    def receive_lifecycle(self, message: VesselLifecycle) -> None:
        """Bind future commands to the bridge's concrete active-vessel identity."""
        active = message.state in (
            VesselLifecycle.STATE_ACTIVE,
            VesselLifecycle.STATE_CHANGED,
        )
        self.session.observe(message.vessel_id, active, message.generation)
        if self.session.fault:
            self._stop(self.session.fault)
        self.lease.observe_vessel(message.vessel_id, active, message.generation)

    def receive_authority(self, message: ControlAuthorityState) -> None:
        """Accept ownership only after the KSP-side authority aggregate confirms it."""
        lost = self.lease.observe_authority(
            message.vessel_id,
            message.controller_id,
            message.lease_id,
            message.state == ControlAuthorityState.STATE_OWNED,
        )

        if lost:
            self._stop("authority_lost")

    def _stop(self, reason):
        self.session.stop(reason)
        self.target = (0.0, 0.0, 0.0)
        self.last_command = None
        now = self.get_clock().now()
        if self.lease.owned:
            self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            self._release_authority(now)
        self.get_logger().warning(
            f"Stopped: {self.session.fault}; cancel navigation and restart the launch",
            throttle_duration_sec=5.0,
        )

    def receive_command(self, message: Twist) -> None:
        """Store the latest finite Nav2 velocity target."""
        if not self.session.ready:
            return
        target = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(value) for value in target):
            self._stop("invalid_cmd_vel")
            return
        self.target = target
        self.last_command = self.get_clock().now()

    def receive_wrench_feedback(self, message: WrenchFeedback) -> None:
        """Pause briefly when KSP's continuous-actuation guard trips."""
        if (
            message.vessel_id == self.lease.vessel_id
            and message.lease_id == self.lease.lease_id
            and message.controller_id == self.lease.controller_id
            and "continuous_actuation_limit" in message.reason.split(",")
            and self.safety_cooldown_until is None
        ):
            self.safety_cooldown_until = self.get_clock().now() + self.safety_cooldown

    def receive_odometry(self, message: Odometry) -> None:
        """Store the latest finite body-frame LiDAR velocity estimate."""
        if message.child_frame_id and message.child_frame_id != self.odom_base_frame:
            self.get_logger().warning(
                "Ignoring odometry for "
                f"{message.child_frame_id}; expected {self.odom_base_frame}",
                throttle_duration_sec=5.0,
            )
            return
        measured = (
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.angular.z,
        )
        if not all(math.isfinite(value) for value in measured):
            self.get_logger().warning("Ignoring non-finite LiDAR odometry")
            return
        self.measured = measured
        self.last_odometry = self.get_clock().now()

    def control(self) -> None:
        """Publish bounded wrench feedback while command and odometry are fresh."""
        if not self.session.ready or self.last_command is None:
            return
        now = self.get_clock().now()
        command_age = now - self.last_command
        if command_age > self.release_timeout:
            self._release_authority(now)
            return
        if self.last_odometry is None or now - self.last_odometry > self.odom_timeout:
            self._stop("odometry_timeout")
            return
        self._maintain_authority(now)
        if not self.lease.owned:
            return

        if self.safety_cooldown_until is not None:
            if now < self.safety_cooldown_until:
                self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
                return
            self.safety_cooldown_until = None

        target = self.target if command_age <= self.command_timeout else (0.0, 0.0, 0.0)
        force, torque = planar_wrench(
            target,
            self.measured,
            self.linear_gain,
            self.angular_gain,
            self.max_force,
            self.max_torque,
        )
        force = rotate_planar(force, self.nav_to_body_yaw)
        self._publish_wrench(now, force, torque)

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
        message.priority = self.control_priority
        message.lease_duration_sec = self.lease_duration_sec
        message.suppress_sas = self.suppress_sas
        message.sequence = self.sequence
        self.authority_publisher.publish(message)

    def _publish_wrench(self, now: Time, force, torque) -> None:
        self.sequence += 1
        message = BodyWrenchCommand()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self.wrench_frame
        message.vessel_id = self.lease.vessel_id
        message.controller_id = self.lease.controller_id
        message.lease_id = self.lease.lease_id
        message.sequence = self.sequence
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = force
        message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z = torque
        message.timeout_sec = self.wrench_timeout_sec
        self.publisher.publish(message)

    def destroy_node(self) -> bool:
        if rclpy.ok():
            now = self.get_clock().now()
            if self.lease.owned:
                self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            self._release_authority(now)
        return super().destroy_node()


def main(args=None) -> None:
    """Run the planar wrench controller node."""
    rclpy.init(args=args)
    node = PlanarWrenchController()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
