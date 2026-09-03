"""LiDAR-guided six-degree-of-freedom debris orbit demo node."""

from __future__ import annotations

import json
import math
import os
import struct
from collections import deque
from pathlib import Path
from typing import Deque, Dict, Iterable, List, Optional, Tuple

from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from ksp_ros2_interfaces.msg import ControlSetpoint
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .core import (
    Vector3,
    add,
    bounding_box_center,
    body_orientation_for_sensor_direction,
    detumble_required,
    dot,
    euclidean_clusters,
    integrate_world_orientation,
    make_orbit_basis,
    norm,
    normalize,
    png_bytes,
    quaternion_error_vector,
    rotate_vector,
    safe_filename_component,
    search_direction,
    scale,
    subtract,
    transform_point,
    unwrap_angle,
    vessel_topics,
    voxel_downsample,
)


_POINT_FIELD_FORMATS: Dict[int, Tuple[str, int]] = {
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


class DebrisOrbitNode(Node):
    """Detect one LiDAR cluster, orbit it, point at it, and save indexed images."""

    def __init__(self) -> None:
        super().__init__("debris_orbit")
        self._declare_parameters()

        self.demo_instance_id = self._string("demo_instance_id")
        prefix = self._string("vessel_topic_prefix").rstrip("/")
        lidar_id = self._string("lidar_sensor_id").strip("/")
        camera_id = self._string("camera_sensor_id").strip("/")
        default_topics = vessel_topics(
            prefix, lidar_id, camera_id, self.demo_instance_id
        )
        lidar_topic = self._string("lidar_topic") or default_topics.lidar_points
        camera_topic = self._string("camera_topic") or default_topics.camera_image
        pose_topic = self._string("pose_topic") or default_topics.ground_truth_pose
        twist_topic = self._string("twist_topic") or default_topics.ground_truth_twist
        setpoint_topic = self._string("setpoint_topic") or default_topics.control_setpoint
        status_topic = self._string("status_topic") or default_topics.demo_status

        self.world_frame = self._string("world_frame")
        self.body_frame = self._string("body_frame")
        self.enabled = bool(self.get_parameter("enabled").value)
        self.orbit_radius = self._positive("orbit_radius")
        self.angular_speed = math.radians(self._positive("angular_speed_deg_s"))
        self.orbit_direction = 1 if int(self.get_parameter("orbit_direction").value) >= 0 else -1
        plane_normal = tuple(self.get_parameter("orbit_plane_normal").value)
        self.plane_normal = normalize(plane_normal)  # type: ignore[arg-type]
        self.detumble_enter_rate = math.radians(
            self._positive("detumble_enter_rate_deg_s")
        )
        self.detumble_exit_rate = math.radians(
            self._positive("detumble_exit_rate_deg_s")
        )
        if self.detumble_exit_rate >= self.detumble_enter_rate:
            raise ValueError(
                "detumble_exit_rate_deg_s must be less than "
                "detumble_enter_rate_deg_s"
            )
        self.search_start_delay = self._nonnegative("search_start_delay_sec")
        self.search_yaw_amplitude = math.radians(self._positive("search_yaw_amplitude_deg"))
        self.search_pitch_amplitude = math.radians(self._positive("search_pitch_amplitude_deg"))
        self.search_period = self._positive("search_period_sec")
        self.search_memory_timeout = Duration(
            seconds=self._positive("search_memory_timeout_sec")
        )
        self.arrival_position_tolerance = self._positive(
            "arrival_position_tolerance"
        )
        self.arrival_speed_tolerance = self._positive("arrival_speed_tolerance")
        self.arrival_attitude_tolerance = math.radians(
            self._positive("arrival_attitude_tolerance_deg")
        )
        self.cluster_tolerance = self._positive("cluster_tolerance")
        self.cluster_min_points = int(self.get_parameter("cluster_min_points").value)
        self.voxel_size = self._positive("voxel_size")
        self.max_points = int(self.get_parameter("max_points").value)
        self.min_target_range = self._positive("min_target_range")
        self.max_target_range = self._positive("max_target_range")
        self.target_cluster_index = max(0, int(self.get_parameter("target_cluster_index").value))
        self.target_max_jump = self._positive("target_max_jump")
        target_filter_alpha = float(self.get_parameter("target_filter_alpha").value)
        self.target_filter_alpha = min(1.0, max(0.01, target_filter_alpha))
        velocity_filter_alpha = float(
            self.get_parameter("target_velocity_filter_alpha").value
        )
        self.target_velocity_filter_alpha = min(
            1.0, max(0.01, velocity_filter_alpha)
        )
        self.target_center_offset = float(self.get_parameter("target_center_offset").value)
        self.target_acquisition_samples = max(
            1, int(self.get_parameter("target_acquisition_samples").value)
        )
        self.state_timeout = Duration(seconds=self._positive("state_timeout_sec"))
        self.target_timeout = Duration(seconds=self._positive("target_timeout_sec"))
        self.transform_wait_timeout = Duration(
            seconds=self._positive("transform_wait_timeout_sec")
        )
        self.cloud_queue_size = max(
            1, int(self.get_parameter("cloud_queue_size").value)
        )
        self.capture_step = math.radians(self._positive("capture_step_deg"))
        self.capture_count = max(1, int(self.get_parameter("capture_count").value))
        self.capture_tolerance = math.radians(self._positive("capture_tolerance_deg"))
        self.stop_after_capture = bool(self.get_parameter("stop_after_capture").value)
        output_directory = os.path.expandvars(
            os.path.expanduser(self._string("output_directory"))
        )
        self.output_directory = Path(output_directory).resolve()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose: Optional[PoseStamped] = None
        self.twist: Optional[TwistStamped] = None
        self.pose_received: Optional[Time] = None
        self.twist_received: Optional[Time] = None
        self.target: Optional[Vector3] = None
        self.target_velocity: Vector3 = (0.0, 0.0, 0.0)
        self.target_received: Optional[Time] = None
        self.target_stamp: Optional[Time] = None
        self.target_observations = 0
        self.last_target: Optional[Vector3] = None
        self.last_target_velocity: Vector3 = (0.0, 0.0, 0.0)
        self.last_target_stamp: Optional[Time] = None
        self.last_target_received: Optional[Time] = None
        self.lidar_mount_rotation = (0.0, 0.0, 0.0, 1.0)
        self.lidar_mount_known = False
        self.search_started: Optional[Time] = None
        self.search_reference_direction: Optional[Vector3] = None
        self.detumbling = False
        self.cloud_queue: Deque[Tuple[PointCloud2, Time]] = deque()
        self.orbit_started: Optional[Time] = None
        self.basis_u: Optional[Vector3] = None
        self.basis_v: Optional[Vector3] = None
        self.actual_angle: Optional[float] = None
        self.next_capture_angle = 0.0
        self.pending_capture: Optional[Tuple[int, float]] = None
        self.saved_captures = 0
        self.complete = False
        self.last_status = ""

        self.setpoint_publisher = self.create_publisher(ControlSetpoint, setpoint_topic, 10)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(String, status_topic, status_qos)
        self.create_subscription(
            PointCloud2, lidar_topic, self.receive_cloud, qos_profile_sensor_data
        )
        self.create_subscription(Image, camera_topic, self.receive_image, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, pose_topic, self.receive_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            TwistStamped, twist_topic, self.receive_twist, qos_profile_sensor_data
        )
        rate = self._positive("control_rate_hz")
        self.create_timer(1.0 / rate, self.control)
        transform_retry_rate = self._positive("transform_retry_rate_hz")
        self.create_timer(1.0 / transform_retry_rate, self.process_cloud_queue)
        self.get_logger().info(
            f"demo_instance={self.demo_instance_id} lidar={lidar_topic} "
            f"camera={camera_topic} setpoint={setpoint_topic} "
            f"status={status_topic} enabled={self.enabled}"
        )
        self._publish_status("waiting_for_sensor_data")

    def _declare_parameters(self) -> None:
        parameters = {
            "enabled": False,
            "demo_instance_id": "demo_vehicle",
            "lidar_sensor_id": "front_lidar",
            "camera_sensor_id": "orbit_camera",
            "vessel_topic_prefix": "/ksp_vessel",
            "lidar_topic": "",
            "camera_topic": "",
            "pose_topic": "",
            "twist_topic": "",
            "setpoint_topic": "",
            "status_topic": "",
            "world_frame": "ground_truth_enu",
            "body_frame": "base_link",
            "orbit_radius": 15.0,
            "angular_speed_deg_s": 3.0,
            "orbit_direction": 1,
            "orbit_plane_normal": [0.0, 0.0, 1.0],
            "detumble_enter_rate_deg_s": 6.0,
            "detumble_exit_rate_deg_s": 2.0,
            "search_start_delay_sec": 0.3,
            "search_yaw_amplitude_deg": 8.0,
            "search_pitch_amplitude_deg": 4.0,
            "search_period_sec": 8.0,
            "search_memory_timeout_sec": 10.0,
            "arrival_position_tolerance": 0.75,
            "arrival_speed_tolerance": 0.25,
            "arrival_attitude_tolerance_deg": 5.0,
            "cluster_tolerance": 1.5,
            "cluster_min_points": 8,
            "voxel_size": 0.35,
            "max_points": 4000,
            "min_target_range": 1.0,
            "max_target_range": 150.0,
            "target_cluster_index": 0,
            "target_max_jump": 8.0,
            "target_filter_alpha": 0.25,
            "target_velocity_filter_alpha": 0.35,
            "target_center_offset": 0.0,
            "target_acquisition_samples": 3,
            "state_timeout_sec": 0.5,
            "target_timeout_sec": 1.0,
            "transform_wait_timeout_sec": 1.0,
            "transform_retry_rate_hz": 50.0,
            "cloud_queue_size": 20,
            "control_rate_hz": 20.0,
            "capture_step_deg": 30.0,
            "capture_count": 12,
            "capture_tolerance_deg": 2.0,
            "stop_after_capture": True,
            "output_directory": "debris_orbit_captures",
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"parameter {name} must be finite and greater than zero")
        return value

    def _nonnegative(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"parameter {name} must be finite and nonnegative")
        return value

    def receive_pose(self, message: PoseStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.world_frame:
            self.get_logger().warning(
                f"Ignoring pose in {message.header.frame_id}; expected {self.world_frame}",
                throttle_duration_sec=5.0,
            )
            return
        self.pose = message
        self.pose_received = self.get_clock().now()

    def receive_twist(self, message: TwistStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.world_frame:
            self.get_logger().warning(
                f"Ignoring twist in {message.header.frame_id}; expected {self.world_frame}",
                throttle_duration_sec=5.0,
            )
            return
        self.twist = message
        self.twist_received = self.get_clock().now()

    def receive_cloud(self, message: PointCloud2) -> None:
        if len(self.cloud_queue) >= self.cloud_queue_size:
            self.cloud_queue.popleft()
            self.get_logger().warning(
                "LiDAR cloud queue is full; dropped the oldest cloud",
                throttle_duration_sec=5.0,
            )
        self.cloud_queue.append((message, self.get_clock().now()))

    def process_cloud_queue(self) -> None:
        """Wait asynchronously for cloud-time transforms without blocking TF callbacks."""
        if not self.cloud_queue:
            return
        message, queued_at = self.cloud_queue[0]
        now = self.get_clock().now()
        stamp = Time.from_msg(message.header.stamp)
        pose_ready = self._state_reaches_stamp(stamp)
        # The LiDAR-to-body chain is a /tf_static extrinsic/model transform.
        # Use the latest value so a model refresh cannot make an otherwise
        # valid point cloud wait for a timestamped dynamic edge. World motion
        # is evaluated at `stamp` from Ground Truth below.
        transform_time = Time()
        transform_ready = bool(message.header.frame_id) and self.tf_buffer.can_transform(
            self.body_frame,
            message.header.frame_id,
            transform_time,
            timeout=Duration(seconds=0.0),
        )
        if not pose_ready or not transform_ready:
            if now - queued_at > self.transform_wait_timeout:
                self.cloud_queue.popleft()
                missing = []
                if not pose_ready:
                    missing.append("Ground Truth state")
                if not transform_ready:
                    missing.append(
                        f"TF {self.body_frame} <- {message.header.frame_id or '<empty>'}"
                    )
                self.get_logger().warning(
                    "Dropped a LiDAR cloud after waiting "
                    f"{self.transform_wait_timeout.nanoseconds * 1.0e-9:.2f}s for "
                    + " and ".join(missing),
                    throttle_duration_sec=5.0,
                )
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.body_frame,
                message.header.frame_id,
                transform_time,
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return
        self.cloud_queue.popleft()
        self._process_cloud(message, stamp, transform)

    def _process_cloud(
        self, message: PointCloud2, stamp: Time, transform: TransformStamped
    ) -> None:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        transform_translation = (translation.x, translation.y, translation.z)
        transform_rotation = (rotation.x, rotation.y, rotation.z, rotation.w)
        # base_link <- sensor: preserve the real LiDAR mounting orientation so
        # guidance aims the sensor axis rather than assuming body +X.
        self.lidar_mount_rotation = transform_rotation
        self.lidar_mount_known = True
        points = [
            point
            for point in point_cloud_xyz(message)
            if self.min_target_range <= norm(point) <= self.max_target_range
        ]
        sampled = voxel_downsample(points, self.voxel_size, self.max_points)
        clusters = euclidean_clusters(sampled, self.cluster_tolerance, self.cluster_min_points)
        if not clusters:
            return
        candidates_body: List[Vector3] = []
        for cluster in clusters:
            center_sensor = bounding_box_center(cluster)
            if self.target_center_offset:
                center_sensor = add(
                    center_sensor,
                    scale(normalize(center_sensor), self.target_center_offset),
                )
            candidates_body.append(
                transform_point(transform_translation, transform_rotation, center_sensor)
            )

        pose_position, pose_orientation = self._body_pose_at(stamp)
        candidates = [
            add(pose_position, rotate_vector(pose_orientation, center_body))
            for center_body in candidates_body
        ]

        received = self.get_clock().now()
        if self.target is None:
            if self.twist is None:
                return
            if self.target_cluster_index >= len(candidates):
                return
            selected = candidates[self.target_cluster_index]
            self.target_velocity = self._twist_linear()
            self.target_observations = 1
        else:
            assert self.target_stamp is not None
            elapsed = (stamp - self.target_stamp).nanoseconds * 1.0e-9
            if elapsed <= 1.0e-3:
                return
            predicted = add(self.target, scale(self.target_velocity, elapsed))
            measurement = min(
                candidates, key=lambda candidate: norm(subtract(candidate, predicted))
            )
            innovation = subtract(measurement, predicted)
            if norm(innovation) > self.target_max_jump:
                return
            selected = add(
                predicted,
                scale(innovation, self.target_filter_alpha),
            )
            measured_velocity = scale(subtract(selected, self.target), 1.0 / elapsed)
            self.target_velocity = add(
                scale(
                    self.target_velocity, 1.0 - self.target_velocity_filter_alpha
                ),
                scale(measured_velocity, self.target_velocity_filter_alpha),
            )
            self.target_observations += 1
        self.target = selected
        self.target_received = received
        self.target_stamp = stamp
        self.last_target = selected
        self.last_target_velocity = self.target_velocity
        self.last_target_stamp = stamp
        self.last_target_received = received
        self.search_started = None
        self.search_reference_direction = None

    def receive_image(self, message: Image) -> None:
        if self.pending_capture is None or self.complete:
            return
        capture_index, angle = self.pending_capture
        try:
            image_data = png_bytes(
                message.width, message.height, message.encoding, message.step, bytes(message.data)
            )
            self.output_directory.mkdir(parents=True, exist_ok=True)
            platform_name = safe_filename_component(self.demo_instance_id, "vehicle")
            filename = (
                f"{platform_name}_detected_debris_"
                f"{int(round(math.degrees(angle))) % 360:03d}deg_{capture_index:02d}.png"
            )
            path = self.output_directory / filename
            path.write_bytes(image_data)
        except (OSError, ValueError) as error:
            self.get_logger().error(f"Could not save capture: {error}")
            return
        self.pending_capture = None
        self.saved_captures += 1
        self.get_logger().info(f"Saved capture {self.saved_captures}/{self.capture_count}: {path}")
        if self.saved_captures >= self.capture_count:
            self.complete = True
            self._publish_status("complete", image_path=str(path))

    def _initialize_orbit(self, target: Vector3) -> bool:
        if self.pose is None:
            return False
        position = self._pose_position()
        radial = subtract(position, target)
        self.basis_u, self.basis_v, self.plane_normal = make_orbit_basis(
            radial, self.plane_normal, self.orbit_direction
        )
        self._publish_status("approaching_orbit")
        return True

    def control(self) -> None:
        now = self.get_clock().now()
        if not self.enabled:
            self._stop_detumble()
            self._publish_idle_setpoint(now)
            self._publish_status("disabled")
            return
        if self.complete and self.stop_after_capture:
            self._publish_idle_setpoint(now)
            return
        if not self._state_is_fresh(now):
            self._stop_detumble()
            self._publish_idle_setpoint(now)
            self._publish_status("waiting_for_fresh_pose_and_twist")
            return
        if self._update_detumble_mode(now):
            return
        if (
            self.target is None
            or self.target_received is None
            or now - self.target_received > self.target_timeout
        ):
            self._reset_target_tracking()
            self._publish_search_setpoint(now)
            return
        if self.target_observations < self.target_acquisition_samples:
            target = self._predicted_target(now)
            if target is not None:
                self._publish_target_attitude_setpoint(now, target)
            self._publish_status(
                "acquiring_target",
                observations=self.target_observations,
                required_observations=self.target_acquisition_samples,
            )
            return
        target = self._predicted_target(now)
        if target is None:
            self._publish_search_setpoint(now)
            return
        if self.basis_u is None and not self._initialize_orbit(target):
            self._publish_idle_setpoint(now)
            return
        assert self.pose is not None and self.twist is not None and self.target is not None
        assert self.basis_u is not None and self.basis_v is not None

        if self.orbit_started is None:
            desired_angle = 0.0
        else:
            elapsed = (now - self.orbit_started).nanoseconds * 1.0e-9
            desired_angle = self.angular_speed * elapsed
        cosine = math.cos(desired_angle)
        sine = math.sin(desired_angle)
        radial_unit = add(scale(self.basis_u, cosine), scale(self.basis_v, sine))
        tangent_unit = add(scale(self.basis_u, -sine), scale(self.basis_v, cosine))
        desired_position = add(target, scale(radial_unit, self.orbit_radius))
        desired_velocity = (
            self.target_velocity
            if self.orbit_started is None
            else add(
                self.target_velocity,
                scale(tangent_unit, self.orbit_radius * self.angular_speed),
            )
        )

        position = self._pose_position()
        velocity = self._twist_linear()
        position_error = subtract(desired_position, position)
        current_q = self._pose_quaternion()
        desired_q = body_orientation_for_sensor_direction(
            current_q, self.lidar_mount_rotation, subtract(target, position)
        )
        attitude_error = quaternion_error_vector(desired_q, current_q)
        self._publish_setpoint(
            now,
            ControlSetpoint.MODE_SIX_DOF,
            desired_position,
            desired_q,
            desired_velocity,
            (0.0, 0.0, 0.0)
            if self.orbit_started is None
            else scale(
                self.plane_normal,
                self.angular_speed * self.orbit_direction,
            ),
        )
        if self.orbit_started is None:
            if (
                norm(position_error) <= self.arrival_position_tolerance
                and norm(subtract(velocity, self.target_velocity))
                <= self.arrival_speed_tolerance
                and norm(attitude_error) <= self.arrival_attitude_tolerance
            ):
                self.orbit_started = now
                self.actual_angle = 0.0
                self.next_capture_angle = self.capture_step
                self.pending_capture = (0, 0.0)
                self._publish_status("orbiting")
            return
        self._update_capture_progress(position, target)

    def _publish_target_attitude_setpoint(self, now: Time, target: Vector3) -> None:
        direction = subtract(target, self._pose_position())
        desired_q = body_orientation_for_sensor_direction(
            self._pose_quaternion(), self.lidar_mount_rotation, direction
        )
        self._publish_setpoint(
            now,
            ControlSetpoint.MODE_ATTITUDE_HOLD,
            self._pose_position(),
            desired_q,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )

    def _publish_search_setpoint(self, now: Time) -> None:
        if not self.lidar_mount_known:
            self._publish_setpoint(
                now,
                ControlSetpoint.MODE_ATTITUDE_HOLD,
                self._pose_position(),
                self._pose_quaternion(),
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            )
            self._publish_status("waiting_for_lidar_transform")
            return
        if self.search_started is None:
            self.search_started = now
        elapsed = max(0.0, (now - self.search_started).nanoseconds * 1.0e-9)
        if self.search_reference_direction is None:
            remembered = self._remembered_target(now)
            if remembered is not None:
                self.search_reference_direction = subtract(remembered, self._pose_position())
            else:
                sensor_world = rotate_vector(
                    self._pose_quaternion(),
                    rotate_vector(self.lidar_mount_rotation, (1.0, 0.0, 0.0)),
                )
                self.search_reference_direction = sensor_world
        scan_elapsed = max(0.0, elapsed - self.search_start_delay)
        direction = search_direction(
            self.search_reference_direction,
            self.plane_normal,
            scan_elapsed,
            self.search_period,
            0.0 if elapsed < self.search_start_delay else self.search_yaw_amplitude,
            0.0 if elapsed < self.search_start_delay else self.search_pitch_amplitude,
        )
        desired_q = body_orientation_for_sensor_direction(
            self._pose_quaternion(), self.lidar_mount_rotation, direction
        )
        self._publish_setpoint(
            now,
            ControlSetpoint.MODE_ATTITUDE_HOLD,
            self._pose_position(),
            desired_q,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )
        self._publish_status(
            "searching",
            elapsed_sec=round(elapsed, 1),
            remembered_target=self._remembered_target(now) is not None,
        )

    def _update_capture_progress(self, position: Vector3, target: Vector3) -> None:
        assert self.basis_u is not None and self.basis_v is not None
        radial = subtract(position, target)
        wrapped = math.atan2(dot(radial, self.basis_v), dot(radial, self.basis_u))
        if self.actual_angle is None:
            self.actual_angle = wrapped
        else:
            self.actual_angle = unwrap_angle(self.actual_angle, wrapped)
        if self.pending_capture is not None or self.saved_captures >= self.capture_count:
            return
        if self.actual_angle + self.capture_tolerance >= self.next_capture_angle:
            index = self.saved_captures
            self.pending_capture = (index, self.next_capture_angle)
            self.next_capture_angle += self.capture_step

    def _state_is_fresh(self, now: Time) -> bool:
        return bool(
            self.pose is not None
            and self.twist is not None
            and self.pose_received is not None
            and self.twist_received is not None
            and now - self.pose_received <= self.state_timeout
            and now - self.twist_received <= self.state_timeout
        )

    def _update_detumble_mode(self, now: Time) -> bool:
        angular_velocity = self._twist_angular()
        angular_rate = norm(angular_velocity)
        required = detumble_required(
            self.detumbling,
            angular_rate,
            self.detumble_enter_rate,
            self.detumble_exit_rate,
        )
        if not required:
            self._stop_detumble()
            return False

        self.detumbling = True
        self._reset_target_tracking()
        self._publish_setpoint(
            now,
            ControlSetpoint.MODE_DETUMBLE,
            self._pose_position(),
            self._pose_quaternion(),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )
        self._publish_status(
            "detumbling",
            angular_rate_deg_s=round(math.degrees(angular_rate), 1),
        )
        return True

    def _stop_detumble(self) -> None:
        self.detumbling = False

    def _reset_target_tracking(self) -> None:
        self.target = None
        self.target_velocity = (0.0, 0.0, 0.0)
        self.target_received = None
        self.target_stamp = None
        self.target_observations = 0
        self.basis_u = None
        self.basis_v = None
        self.orbit_started = None
        self.actual_angle = None
        self.pending_capture = None

    def _remembered_target(self, now: Time) -> Optional[Vector3]:
        if (
            self.last_target is None
            or self.last_target_stamp is None
            or self.last_target_received is None
            or now - self.last_target_received > self.search_memory_timeout
        ):
            return None
        elapsed = max(0.0, (now - self.last_target_stamp).nanoseconds * 1.0e-9)
        return add(self.last_target, scale(self.last_target_velocity, elapsed))

    def _state_reaches_stamp(self, stamp: Time) -> bool:
        if self.pose is None or self.twist is None:
            return False
        pose_stamp = Time.from_msg(self.pose.header.stamp)
        twist_stamp = Time.from_msg(self.twist.header.stamp)
        return pose_stamp >= stamp and twist_stamp >= stamp

    def _body_pose_at(self, stamp: Time) -> Tuple[Vector3, Tuple[float, float, float, float]]:
        """Back-propagate the newest Ground Truth sample to a queued cloud stamp."""
        assert self.pose is not None and self.twist is not None
        pose_stamp = Time.from_msg(self.pose.header.stamp)
        elapsed = (stamp - pose_stamp).nanoseconds * 1.0e-9
        position = add(self._pose_position(), scale(self._twist_linear(), elapsed))
        orientation = integrate_world_orientation(
            self._pose_quaternion(), self._twist_angular(), elapsed
        )
        return position, orientation

    def _predicted_target(self, now: Time) -> Optional[Vector3]:
        if self.target is None or self.target_stamp is None:
            return None
        elapsed = max(0.0, (now - self.target_stamp).nanoseconds * 1.0e-9)
        return add(self.target, scale(self.target_velocity, elapsed))

    def _pose_position(self) -> Vector3:
        assert self.pose is not None
        p = self.pose.pose.position
        return (p.x, p.y, p.z)

    def _pose_quaternion(self) -> Tuple[float, float, float, float]:
        assert self.pose is not None
        q = self.pose.pose.orientation
        return (q.x, q.y, q.z, q.w)

    def _twist_linear(self) -> Vector3:
        assert self.twist is not None
        v = self.twist.twist.linear
        return (v.x, v.y, v.z)

    def _twist_angular(self) -> Vector3:
        assert self.twist is not None
        v = self.twist.twist.angular
        return (v.x, v.y, v.z)

    def _publish_setpoint(
        self,
        now: Time,
        mode: int,
        position: Vector3,
        orientation: Tuple[float, float, float, float],
        linear_velocity: Vector3,
        angular_velocity: Vector3,
    ) -> None:
        message = ControlSetpoint()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self.world_frame
        message.mode = mode
        message.position.x, message.position.y, message.position.z = position
        (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        ) = orientation
        (
            message.linear_velocity.x,
            message.linear_velocity.y,
            message.linear_velocity.z,
        ) = linear_velocity
        (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        ) = angular_velocity
        self.setpoint_publisher.publish(message)

    def _publish_idle_setpoint(self, now: Time) -> None:
        self._publish_setpoint(
            now,
            ControlSetpoint.MODE_IDLE,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )

    def _publish_status(self, state: str, **extra: object) -> None:
        payload = {
            "state": state,
            "demo_instance_id": self.demo_instance_id,
            "captures": self.saved_captures,
            "capture_count": self.capture_count,
        }
        payload.update(extra)
        serialized = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        if serialized == self.last_status:
            return
        self.last_status = serialized
        message = String()
        message.data = serialized
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DebrisOrbitNode()
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
