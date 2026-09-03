"""ROS2 adapter from Nav2 planar velocity commands to KSP body wrench."""

import math
from typing import Optional

from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from .math_utils import planar_wrench, rotate_planar


class PlanarWrenchController(Node):
    """Track Nav2 planar velocity commands through the bridge body-wrench API."""

    def __init__(self) -> None:
        """Create controller parameters, ROS interfaces, and the control timer."""
        super().__init__("ksp_planar_wrench_controller")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/ksp_nav2/odom")
        self.declare_parameter("body_wrench_topic", "/ksp_vessel/body_wrench")
        self.declare_parameter("odom_base_frame", "nav_base_link")
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
        rate = max(1.0, float(self.get_parameter("control_rate").value))

        self.target = (0.0, 0.0, 0.0)
        self.measured = (0.0, 0.0, 0.0)
        self.last_command: Optional[Time] = None
        self.last_odometry: Optional[Time] = None

        self.publisher = self.create_publisher(
            WrenchStamped, str(self.get_parameter("body_wrench_topic").value), 10
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
        self.timer = self.create_timer(1.0 / rate, self.control)

    def receive_command(self, message: Twist) -> None:
        """Store the latest finite Nav2 velocity target."""
        target = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(value) for value in target):
            self.get_logger().warning("Ignoring non-finite cmd_vel")
            return
        self.target = target
        self.last_command = self.get_clock().now()

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
        if self.last_command is None or self.last_odometry is None:
            return
        now = self.get_clock().now()
        odom_age = now - self.last_odometry
        command_age = now - self.last_command
        if odom_age > self.odom_timeout:
            return
        if command_age > self.command_timeout + self.brake_duration:
            return

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
        message = WrenchStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self.wrench_frame
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = force
        message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z = torque
        self.publisher.publish(message)


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
