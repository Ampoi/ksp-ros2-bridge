"""ROS2 node providing odometry from consecutive 2D LiDAR scans."""

import copy
import math
from typing import Optional

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from pylon_interfaces.msg import VesselLifecycle
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from .session import SessionGuard
from .scan_matching import iterative_closest_point, laser_points, yaw_from_transform


class LaserScanOdometry(Node):
    """Estimate planar odometry using only consecutive 2D LaserScan messages."""

    def __init__(self) -> None:
        """Create publishers, subscription, parameters, and ICP state."""
        super().__init__("pylon_laser_scan_odometry")
        self.session = SessionGuard()
        self.declare_parameter("vessel_lifecycle_topic", "/ksp_vessel/lifecycle")
        self.declare_parameter("scan_topic", "/ksp_vessel/lidar_2d/front_lidar/scan")
        self.declare_parameter("filtered_scan_topic", "/pylon/lidar_slam/scan")
        self.declare_parameter("odom_topic", "/pylon/lidar_slam/odom")
        self.declare_parameter("odom_frame", "pylon_slam_odom")
        self.declare_parameter("base_frame", "pylon_slam_base_link")
        self.declare_parameter("max_points", 720)
        self.declare_parameter("max_iterations", 20)
        self.declare_parameter("max_correspondence_distance", 2.0)
        self.declare_parameter("trim_fraction", 0.8)
        self.declare_parameter("minimum_correspondences", 20)
        self.declare_parameter("convergence_tolerance", 0.0001)
        self.declare_parameter("max_rmse", 0.5)
        self.declare_parameter("max_translation_per_scan", 2.0)
        self.declare_parameter("max_rotation_per_scan", 0.5)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.max_iterations = int(self.get_parameter("max_iterations").value)
        self.max_correspondence_distance = float(
            self.get_parameter("max_correspondence_distance").value
        )
        self.trim_fraction = float(self.get_parameter("trim_fraction").value)
        self.minimum_correspondences = int(
            self.get_parameter("minimum_correspondences").value
        )
        self.convergence_tolerance = float(
            self.get_parameter("convergence_tolerance").value
        )
        self.max_rmse = float(self.get_parameter("max_rmse").value)
        self.max_translation_per_scan = float(
            self.get_parameter("max_translation_per_scan").value
        )
        self.max_rotation_per_scan = float(
            self.get_parameter("max_rotation_per_scan").value
        )

        self.previous_points: Optional[np.ndarray] = None
        self.previous_stamp: Optional[float] = None
        self.previous_delta = np.eye(3, dtype=np.float64)
        self.pose = np.eye(3, dtype=np.float64)

        self.odom_publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self.scan_publisher = self.create_publisher(
            LaserScan, str(self.get_parameter("filtered_scan_topic").value), 10
        )
        self.transform_broadcaster = TransformBroadcaster(self)
        self.scan_subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.receive_scan,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            VesselLifecycle,
            str(self.get_parameter("vessel_lifecycle_topic").value),
            self.receive_lifecycle,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    def receive_lifecycle(self, message):
        active = message.state in (VesselLifecycle.STATE_ACTIVE, VesselLifecycle.STATE_CHANGED)
        self.session.observe(message.vessel_id, active, message.generation)
        if self.session.fault:
            self.get_logger().warning(
                "Flight session changed; restart mapping/navigation to reset its map and odometry",
                throttle_duration_sec=5.0,
            )

    def receive_scan(self, message: LaserScan) -> None:
        """Relay one scan and update LiDAR-only odometry when matching succeeds."""
        if not self.session.ready:
            return

        points = laser_points(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            message.range_min,
            message.range_max,
            self.max_points,
        )
        if len(points) < self.minimum_correspondences:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.previous_points is None or self.previous_stamp is None:
            self.previous_points = points
            self.previous_stamp = stamp
            self.publish_state(message, (0.0, 0.0, 0.0), 0.05)
            return

        dt = stamp - self.previous_stamp
        if dt == 0.0:
            return
        if dt < 0.0 or dt > 2.0:
            self.session.stop("scan_time_discontinuity")
            self.get_logger().warning("Scan time changed; restart mapping/navigation")
            return

        try:
            delta, error, _ = iterative_closest_point(
                points,
                self.previous_points,
                self.previous_delta,
                max_iterations=self.max_iterations,
                max_correspondence_distance=self.max_correspondence_distance,
                trim_fraction=self.trim_fraction,
                minimum_correspondences=self.minimum_correspondences,
                convergence_tolerance=self.convergence_tolerance,
            )
            translation = float(np.linalg.norm(delta[:2, 2]))
            rotation = abs(yaw_from_transform(delta))
            if not np.all(np.isfinite(delta)) or error > self.max_rmse:
                raise ValueError("scan match quality is below the configured limit")
            if translation > self.max_translation_per_scan:
                raise ValueError("scan match translation exceeds the configured limit")
            if rotation > self.max_rotation_per_scan:
                raise ValueError("scan match rotation exceeds the configured limit")
        except ValueError as exc:
            self.get_logger().warning(
                f"LiDAR odometry rejected a scan: {exc}",
                throttle_duration_sec=2.0,
            )
            self.previous_delta = np.eye(3, dtype=np.float64)
            return

        self.pose = self.pose @ delta
        self.previous_delta = delta
        self.previous_points = points
        self.previous_stamp = stamp

        delta_yaw = yaw_from_transform(delta)
        displacement_current = delta[:2, :2].T @ delta[:2, 2]
        velocity = (
            float(displacement_current[0] / dt),
            float(displacement_current[1] / dt),
            float(delta_yaw / dt),
        )
        self.publish_state(message, velocity, max(error * error, 1e-4))

    def publish_state(
        self, scan: LaserScan, velocity: tuple[float, float, float], variance: float
    ) -> None:
        """Publish the current odometry message and its matching TF."""
        yaw = yaw_from_transform(self.pose)
        half_yaw = yaw * 0.5
        quaternion_z = math.sin(half_yaw)
        quaternion_w = math.cos(half_yaw)

        odometry = Odometry()
        odometry.header.stamp = scan.header.stamp
        odometry.header.frame_id = self.odom_frame
        odometry.child_frame_id = self.base_frame
        odometry.pose.pose.position.x = float(self.pose[0, 2])
        odometry.pose.pose.position.y = float(self.pose[1, 2])
        odometry.pose.pose.orientation.z = quaternion_z
        odometry.pose.pose.orientation.w = quaternion_w
        odometry.twist.twist.linear.x = velocity[0]
        odometry.twist.twist.linear.y = velocity[1]
        odometry.twist.twist.angular.z = velocity[2]
        odometry.pose.covariance[0] = variance
        odometry.pose.covariance[7] = variance
        odometry.pose.covariance[14] = 1e6
        odometry.pose.covariance[21] = 1e6
        odometry.pose.covariance[28] = 1e6
        odometry.pose.covariance[35] = variance
        odometry.twist.covariance[0] = variance
        odometry.twist.covariance[7] = variance
        odometry.twist.covariance[14] = 1e6
        odometry.twist.covariance[21] = 1e6
        odometry.twist.covariance[28] = 1e6
        odometry.twist.covariance[35] = variance
        self.odom_publisher.publish(odometry)

        transform = TransformStamped()
        transform.header = odometry.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = float(self.pose[0, 2])
        transform.transform.translation.y = float(self.pose[1, 2])
        transform.transform.rotation.z = quaternion_z
        transform.transform.rotation.w = quaternion_w
        self.transform_broadcaster.sendTransform(transform)
        relayed_scan = copy.deepcopy(scan)
        relayed_scan.header.frame_id = self.base_frame
        self.scan_publisher.publish(relayed_scan)


def main(args=None) -> None:
    """Run the LiDAR odometry node."""
    rclpy.init(args=args)
    node = LaserScanOdometry()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
