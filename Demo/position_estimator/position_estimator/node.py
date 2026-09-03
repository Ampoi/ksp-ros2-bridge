"""ROS2 node estimating 6DoF pose from consecutive 3D LiDAR clouds."""

from __future__ import annotations

import json
import math
import struct
import time
from collections import deque
from typing import Deque, Dict, Iterable, Optional, Tuple

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .core import (
    attitude_drift_angle,
    default_ground_truth_pose_topic,
    default_lidar_topic,
    filter_range,
    interpolate_samples,
    iterative_closest_point,
    matrix_to_quaternion,
    position_drift,
    rotation_angle,
    sanitize_ros_component,
    transform_points,
    twist_from_delta,
    voxel_downsample,
)


Vector3 = Tuple[float, float, float]

_POINT_FIELD_FORMATS = {
    PointField.INT8: ("b", 1),
    PointField.UINT8: ("B", 1),
    PointField.INT16: ("h", 2),
    PointField.UINT16: ("H", 2),
    PointField.INT32: ("i", 4),
    PointField.UINT32: ("I", 4),
    PointField.FLOAT32: ("f", 4),
    PointField.FLOAT64: ("d", 8),
}


def point_cloud_xyz(message: PointCloud2) -> Iterable[Vector3]:
    """Yield (x, y, z) tuples from a PointCloud2 message."""
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        return
    endian = ">" if message.is_bigendian else "<"
    unpackers = []
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.datatype not in _POINT_FIELD_FORMATS:
            return
        code, _ = _POINT_FIELD_FORMATS[field.datatype]
        unpackers.append((field.offset, struct.Struct(endian + code)))
    data = bytes(message.data)
    for row in range(message.height):
        row_offset = row * message.row_step
        for column in range(message.width):
            point_offset = row_offset + column * message.point_step
            try:
                point = tuple(
                    unpacker.unpack_from(data, point_offset + offset)[0]
                    for offset, unpacker in unpackers
                )
            except struct.error:
                return
            yield point  # type: ignore[misc]


class PositionEstimatorNode(Node):
    """Estimate 6DoF odometry using only consecutive 3D LiDAR scans.

    Ground truth is subscribed only to score the estimate (start-aligned
    drift on a dedicated error topic and console echo). It never enters
    the ICP update, so the odometry stays LiDAR-only.
    """

    def __init__(self) -> None:
        super().__init__("position_estimator")
        self.declare_parameter("lidar_sensor_id", "front_lidar")
        self.declare_parameter("vessel_topic_prefix", "/ksp_vessel")
        self.declare_parameter("lidar_topic", "")
        self.declare_parameter("odom_topic", "/ksp_position_estimator/odom")
        self.declare_parameter("status_topic", "/ksp_position_estimator/status")
        self.declare_parameter("error_topic", "/ksp_position_estimator/error")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("evaluate", True)
        self.declare_parameter("ground_truth_timeout_sec", 0.5)
        self.declare_parameter("ground_truth_buffer_size", 120)
        self.declare_parameter("console_echo_period_sec", 2.0)
        self.declare_parameter("odom_frame", "lidar_odom_3d")
        self.declare_parameter("base_frame", "estimator_base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("voxel_size", 0.4)
        self.declare_parameter("max_points", 800)
        self.declare_parameter("min_range", 1.0)
        self.declare_parameter("max_range", 120.0)
        self.declare_parameter("max_iterations", 25)
        self.declare_parameter("max_correspondence_distance", 1.5)
        self.declare_parameter("trim_fraction", 0.8)
        self.declare_parameter("minimum_correspondences", 30)
        self.declare_parameter("convergence_tolerance", 1e-4)
        self.declare_parameter("max_rmse", 0.6)
        self.declare_parameter("max_translation_per_scan", 3.0)
        self.declare_parameter("max_rotation_per_scan_deg", 30.0)

        prefix = str(self.get_parameter("vessel_topic_prefix").value)
        sensor_id = sanitize_ros_component(
            str(self.get_parameter("lidar_sensor_id").value), "lidar_3d"
        )
        default_topic = default_lidar_topic(prefix, sensor_id)
        configured_topic = str(self.get_parameter("lidar_topic").value).strip()
        self.lidar_topic = configured_topic or default_topic
        configured_pose_topic = str(self.get_parameter("pose_topic").value).strip()
        self.pose_topic = configured_pose_topic or default_ground_truth_pose_topic(
            prefix
        )

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.previous_points: Optional[np.ndarray] = None
        self.previous_stamp: Optional[float] = None
        self.previous_delta = np.eye(4, dtype=np.float64)
        self.pose = np.eye(4, dtype=np.float64)
        self.sequence = 0

        # Evaluation-only ground-truth state. Origins latch on the first
        # evaluated update so drift is measured from a shared start.
        buffer_size = max(10, int(self.get_parameter("ground_truth_buffer_size").value))
        self.ground_truth: Deque[Tuple[float, np.ndarray, Tuple[float, float, float, float]]] = deque(
            maxlen=buffer_size
        )
        self.estimated_origin: Optional[np.ndarray] = None
        self.truth_origin: Optional[np.ndarray] = None
        self.estimated_origin_quat: Optional[Tuple[float, float, float, float]] = None
        self.truth_origin_quat: Optional[Tuple[float, float, float, float]] = None
        self.last_echo_wall_time = 0.0

        self.odom_publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.error_publisher = self.create_publisher(
            String, str(self.get_parameter("error_topic").value), 10
        )
        self.transform_broadcaster = TransformBroadcaster(self)
        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.receive_cloud,
            qos_profile_sensor_data,
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.receive_ground_truth,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Subscribing to 3D LiDAR: {self.lidar_topic} "
            f"ground truth (evaluation only): {self.pose_topic}"
        )

    def _icp_params(self) -> dict:
        return {
            "max_iterations": int(self.get_parameter("max_iterations").value),
            "max_correspondence_distance": float(
                self.get_parameter("max_correspondence_distance").value
            ),
            "trim_fraction": float(self.get_parameter("trim_fraction").value),
            "minimum_correspondences": int(
                self.get_parameter("minimum_correspondences").value
            ),
            "convergence_tolerance": float(
                self.get_parameter("convergence_tolerance").value
            ),
        }

    def receive_ground_truth(self, message: PoseStamped) -> None:
        """Buffer one ground-truth pose for evaluation-only drift scoring."""
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        position = np.array(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            dtype=np.float64,
        )
        orientation = (
            float(message.pose.orientation.x),
            float(message.pose.orientation.y),
            float(message.pose.orientation.z),
            float(message.pose.orientation.w),
        )
        if not np.all(np.isfinite(position)) or not all(
            math.isfinite(value) for value in orientation
        ):
            return
        samples = self.ground_truth
        if samples and stamp < samples[-1][0]:
            samples.clear()
        samples.append((stamp, position, orientation))

    def evaluate_estimate(
        self, stamp: float
    ) -> Optional[Dict[str, object]]:
        """Score the current estimate against buffered ground truth.

        Returns a JSON-serializable dict, or None when evaluation is
        disabled or no fresh ground truth is available. Latches the
        start-aligned origins on the first successful evaluation.
        """
        if not bool(self.get_parameter("evaluate").value):
            return None
        samples = self.ground_truth
        if not samples:
            return None
        timeout = float(self.get_parameter("ground_truth_timeout_sec").value)
        newest = samples[-1][0]
        oldest = samples[0][0]
        if stamp > newest + timeout or stamp < oldest - timeout:
            return None
        stamps = np.array([sample[0] for sample in samples], dtype=np.float64)
        positions = np.array([sample[1] for sample in samples], dtype=np.float64)
        quaternions = np.array([sample[2] for sample in samples], dtype=np.float64)
        truth_position, truth_quat = interpolate_samples(
            stamps, positions, quaternions, stamp
        )
        estimated_position = self.pose[:3, 3].copy()
        estimated_quat = matrix_to_quaternion(self.pose[:3, :3])
        if self.estimated_origin is None or self.truth_origin is None:
            self.estimated_origin = estimated_position.copy()
            self.truth_origin = truth_position.copy()
            self.estimated_origin_quat = estimated_quat
            self.truth_origin_quat = truth_quat
        assert self.estimated_origin is not None and self.truth_origin is not None
        assert self.estimated_origin_quat is not None and self.truth_origin_quat is not None
        error_vector, position_error = position_drift(
            estimated_position,
            self.estimated_origin,
            truth_position,
            self.truth_origin,
        )
        attitude_error = attitude_drift_angle(
            estimated_quat,
            self.estimated_origin_quat,
            truth_quat,
            self.truth_origin_quat,
        )
        estimated_displacement = estimated_position - self.estimated_origin
        truth_displacement = truth_position - self.truth_origin
        return {
            "evaluated": True,
            "estimated_position": [float(value) for value in estimated_position],
            "estimated_displacement": [float(value) for value in estimated_displacement],
            "truth_displacement": [float(value) for value in truth_displacement],
            "position_error_vector": [float(value) for value in error_vector],
            "position_error": float(position_error),
            "attitude_error_deg": float(math.degrees(attitude_error)),
        }

    def receive_cloud(self, message: PointCloud2) -> None:
        """Downsample one cloud and update LiDAR-only 6DoF odometry."""
        raw = np.array(list(point_cloud_xyz(message)), dtype=np.float64).reshape(-1, 3)
        filtered = filter_range(
            raw,
            float(self.get_parameter("min_range").value),
            float(self.get_parameter("max_range").value),
        )
        points = voxel_downsample(
            filtered,
            float(self.get_parameter("voxel_size").value),
            int(self.get_parameter("max_points").value),
        )
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

        if points.shape[0] < int(self.get_parameter("minimum_correspondences").value):
            self.get_logger().warning(
                f"Ignoring cloud with only {points.shape[0]} usable points",
                throttle_duration_sec=2.0,
            )
            return

        if self.previous_points is None or self.previous_stamp is None:
            self.previous_points = points
            self.previous_stamp = stamp
            self.publish_state(message, (0.0,) * 6, 0.05, 0.0, 0)
            return

        dt = stamp - self.previous_stamp
        if dt <= 0.0 or dt > 2.0:
            self.previous_points = points
            self.previous_stamp = stamp
            self.previous_delta = np.eye(4, dtype=np.float64)
            self.publish_state(message, (0.0,) * 6, 1.0, 0.0, 0)
            return

        try:
            delta, error, matches = iterative_closest_point(
                points, self.previous_points, self.previous_delta, **self._icp_params()
            )
            translation = float(np.linalg.norm(delta[:3, 3]))
            rotation = rotation_angle(delta)
            if not np.all(np.isfinite(delta)):
                raise ValueError("scan match produced non-finite values")
            if error > float(self.get_parameter("max_rmse").value):
                raise ValueError("scan match quality is below the configured limit")
            if translation > float(self.get_parameter("max_translation_per_scan").value):
                raise ValueError("scan match translation exceeds the configured limit")
            max_rotation = math.radians(
                float(self.get_parameter("max_rotation_per_scan_deg").value)
            )
            if rotation > max_rotation:
                raise ValueError("scan match rotation exceeds the configured limit")
        except ValueError as exc:
            self.get_logger().warning(
                f"LiDAR odometry rejected a cloud: {exc}",
                throttle_duration_sec=2.0,
            )
            self.previous_points = points
            self.previous_stamp = stamp
            self.previous_delta = np.eye(4, dtype=np.float64)
            self.publish_state(message, (0.0,) * 6, 1.0, 0.0, 0)
            return

        self.pose = self.pose @ delta
        self.previous_delta = delta
        self.previous_points = points
        self.previous_stamp = stamp
        velocity = twist_from_delta(delta, dt)
        self.publish_state(
            message, velocity, max(error * error, 1e-4), error, int(matches)
        )

    def publish_state(
        self,
        cloud: PointCloud2,
        velocity: Tuple[float, float, float, float, float, float],
        variance: float,
        rmse: float,
        matches: int,
    ) -> None:
        """Publish odometry, TF, JSON status, and the evaluation-only error."""
        # Use a separate frame tree so this estimate never competes with the
        # bridge's ground_truth_enu -> base_link transform.
        quat = matrix_to_quaternion(self.pose[:3, :3])

        odometry = Odometry()
        odometry.header.stamp = cloud.header.stamp
        odometry.header.frame_id = self.odom_frame
        odometry.child_frame_id = self.base_frame
        odometry.pose.pose.position.x = float(self.pose[0, 3])
        odometry.pose.pose.position.y = float(self.pose[1, 3])
        odometry.pose.pose.position.z = float(self.pose[2, 3])
        odometry.pose.pose.orientation.x = quat[0]
        odometry.pose.pose.orientation.y = quat[1]
        odometry.pose.pose.orientation.z = quat[2]
        odometry.pose.pose.orientation.w = quat[3]
        odometry.twist.twist.linear.x = velocity[0]
        odometry.twist.twist.linear.y = velocity[1]
        odometry.twist.twist.linear.z = velocity[2]
        odometry.twist.twist.angular.x = velocity[3]
        odometry.twist.twist.angular.y = velocity[4]
        odometry.twist.twist.angular.z = velocity[5]
        for index in (0, 7, 14, 21, 28, 35):
            odometry.pose.covariance[index] = variance
            odometry.twist.covariance[index] = variance
        self.odom_publisher.publish(odometry)

        if bool(self.get_parameter("publish_tf").value):
            transform = TransformStamped()
            transform.header = odometry.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = float(self.pose[0, 3])
            transform.transform.translation.y = float(self.pose[1, 3])
            transform.transform.translation.z = float(self.pose[2, 3])
            transform.transform.rotation.x = quat[0]
            transform.transform.rotation.y = quat[1]
            transform.transform.rotation.z = quat[2]
            transform.transform.rotation.w = quat[3]
            self.transform_broadcaster.sendTransform(transform)

        self.sequence += 1
        stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
        evaluation = self.evaluate_estimate(stamp)
        if evaluation is not None:
            error_message = String()
            error_message.data = json.dumps(
                {
                    "sequence": self.sequence,
                    "stamp": float(stamp),
                    "rmse": float(rmse),
                    "matches": int(matches),
                    **evaluation,
                }
            )
            self.error_publisher.publish(error_message)
        status = String()
        status.data = json.dumps(
            {
                "sequence": self.sequence,
                "points": int(
                    0 if self.previous_points is None else self.previous_points.shape[0]
                ),
                "rmse": float(rmse),
                "matches": int(matches),
                "variance": float(variance),
                "position": [float(self.pose[i, 3]) for i in range(3)],
                "quaternion": [float(value) for value in quat],
                "position_error": evaluation.get("position_error")
                if evaluation is not None
                else None,
                "attitude_error_deg": evaluation.get("attitude_error_deg")
                if evaluation is not None
                else None,
            }
        )
        self.status_publisher.publish(status)
        self.echo_estimate(evaluation, rmse, matches)

    def echo_estimate(
        self,
        evaluation: Optional[Dict[str, object]],
        rmse: float,
        matches: int,
    ) -> None:
        """Echo the estimated position (and drift error, when scored) to the console."""
        period = float(self.get_parameter("console_echo_period_sec").value)
        now_wall = time.monotonic()
        if period > 0.0 and now_wall - self.last_echo_wall_time < period:
            return
        self.last_echo_wall_time = now_wall
        position = [float(self.pose[i, 3]) for i in range(3)]
        if evaluation is None:
            self.get_logger().info(
                f"est=[{position[0]:.2f} {position[1]:.2f} {position[2]:.2f}] "
                f"rmse={rmse:.3f} matches={matches} "
                "(ground truth not yet available for error scoring)"
            )
            return
        self.get_logger().info(
            f"est=[{position[0]:.2f} {position[1]:.2f} {position[2]:.2f}] "
            f"drift={float(evaluation['position_error']):.3f}m "
            f"att_err={float(evaluation['attitude_error_deg']):.2f}deg "
            f"rmse={rmse:.3f} matches={matches}"
        )


def main(args=None) -> None:
    """Run the 3D LiDAR position estimator node."""
    rclpy.init(args=args)
    node = PositionEstimatorNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
