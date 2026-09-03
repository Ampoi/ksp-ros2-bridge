"""LiDAR-guided six-degree-of-freedom debris orbit demo node."""

from __future__ import annotations

import json
import math
import os
import struct
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .core import (
    Vector3,
    add,
    bounding_box_center,
    clamp_norm,
    dot,
    euclidean_clusters,
    look_at_quaternion,
    make_orbit_basis,
    norm,
    normalize,
    png_bytes,
    quaternion_conjugate,
    quaternion_error_vector,
    rotate_vector,
    safe_filename_component,
    scale,
    subtract,
    transform_point,
    unwrap_angle,
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

        self.platform_id = self._string("platform_id")
        prefix = self._string("vessel_topic_prefix").rstrip("/")
        lidar_id = self._string("lidar_sensor_id").strip("/")
        camera_id = self._string("camera_sensor_id").strip("/")
        lidar_topic = self._string("lidar_topic") or f"{prefix}/lidar_3d/{lidar_id}/points"
        camera_topic = self._string("camera_topic") or f"{prefix}/camera/{camera_id}/image_raw"

        self.world_frame = self._string("world_frame")
        self.body_frame = self._string("body_frame")
        self.enabled = bool(self.get_parameter("enabled").value)
        self.orbit_radius = self._positive("orbit_radius")
        self.angular_speed = math.radians(self._positive("angular_speed_deg_s"))
        self.orbit_direction = 1 if int(self.get_parameter("orbit_direction").value) >= 0 else -1
        plane_normal = tuple(self.get_parameter("orbit_plane_normal").value)
        self.plane_normal = normalize(plane_normal)  # type: ignore[arg-type]
        self.position_kp = self._positive("position_kp")
        self.velocity_kd = self._positive("velocity_kd")
        self.attitude_kp = self._positive("attitude_kp")
        self.angular_kd = self._positive("angular_kd")
        self.max_force = self._positive("max_force")
        self.max_torque = self._positive("max_torque")
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
        self.state_timeout = Duration(seconds=self._positive("state_timeout_sec"))
        self.target_timeout = Duration(seconds=self._positive("target_timeout_sec"))
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
        self.orbit_started: Optional[Time] = None
        self.basis_u: Optional[Vector3] = None
        self.basis_v: Optional[Vector3] = None
        self.actual_angle: Optional[float] = None
        self.next_capture_angle = 0.0
        self.pending_capture: Optional[Tuple[int, float]] = None
        self.saved_captures = 0
        self.complete = False
        self.last_status = ""

        self.wrench_publisher = self.create_publisher(
            WrenchStamped, self._string("body_wrench_topic"), 10
        )
        status_topic = f"/debris_orbit/{self.platform_id}/status"
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.create_subscription(
            PointCloud2, lidar_topic, self.receive_cloud, qos_profile_sensor_data
        )
        self.create_subscription(Image, camera_topic, self.receive_image, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, self._string("pose_topic"), self.receive_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            TwistStamped, self._string("twist_topic"), self.receive_twist, qos_profile_sensor_data
        )
        rate = self._positive("control_rate_hz")
        self.create_timer(1.0 / rate, self.control)
        self.get_logger().info(
            f"platform={self.platform_id} lidar={lidar_topic} "
            f"camera={camera_topic} enabled={self.enabled}"
        )
        self._publish_status("waiting_for_sensor_data")

    def _declare_parameters(self) -> None:
        parameters = {
            "enabled": False,
            "platform_id": "demo_vehicle",
            "lidar_sensor_id": "front_lidar",
            "camera_sensor_id": "orbit_camera",
            "vessel_topic_prefix": "/ksp_vessel",
            "lidar_topic": "",
            "camera_topic": "",
            "pose_topic": "/ksp_vessel/ground_truth/pose",
            "twist_topic": "/ksp_vessel/ground_truth/twist",
            "body_wrench_topic": "/ksp_vessel/body_wrench",
            "world_frame": "ground_truth_enu",
            "body_frame": "base_link",
            "orbit_radius": 15.0,
            "angular_speed_deg_s": 3.0,
            "orbit_direction": 1,
            "orbit_plane_normal": [0.0, 0.0, 1.0],
            "position_kp": 120.0,
            "velocity_kd": 350.0,
            "attitude_kp": 800.0,
            "angular_kd": 300.0,
            "max_force": 5000.0,
            "max_torque": 3000.0,
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
            "state_timeout_sec": 0.5,
            "target_timeout_sec": 1.0,
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
        points = [
            point
            for point in point_cloud_xyz(message)
            if self.min_target_range <= norm(point) <= self.max_target_range
        ]
        sampled = voxel_downsample(points, self.voxel_size, self.max_points)
        clusters = euclidean_clusters(sampled, self.cluster_tolerance, self.cluster_min_points)
        if not clusters:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"LiDAR transform unavailable: {error}", throttle_duration_sec=5.0
            )
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        transform_translation = (translation.x, translation.y, translation.z)
        transform_rotation = (rotation.x, rotation.y, rotation.z, rotation.w)
        candidates: List[Vector3] = []
        for cluster in clusters:
            center_sensor = bounding_box_center(cluster)
            if self.target_center_offset:
                center_sensor = add(
                    center_sensor,
                    scale(normalize(center_sensor), self.target_center_offset),
                )
            candidates.append(
                transform_point(transform_translation, transform_rotation, center_sensor)
            )

        received = self.get_clock().now()
        if self.target is None:
            if self.twist is None:
                return
            if self.target_cluster_index >= len(candidates):
                return
            selected = candidates[self.target_cluster_index]
            self.target_velocity = self._twist_linear()
        else:
            assert self.target_received is not None
            elapsed = max(
                1.0e-3, (received - self.target_received).nanoseconds * 1.0e-9
            )
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
        self.target = selected
        self.target_received = received

    def receive_image(self, message: Image) -> None:
        if self.pending_capture is None or self.complete:
            return
        capture_index, angle = self.pending_capture
        try:
            image_data = png_bytes(
                message.width, message.height, message.encoding, message.step, bytes(message.data)
            )
            self.output_directory.mkdir(parents=True, exist_ok=True)
            platform_name = safe_filename_component(self.platform_id, "vehicle")
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
            self._publish_status("disabled")
            return
        if self.complete and self.stop_after_capture:
            self._publish_zero_wrench(now)
            return
        if not self._state_is_fresh(now):
            self._publish_status("waiting_for_fresh_pose_and_twist")
            return
        if (
            self.target is None
            or self.target_received is None
            or now - self.target_received > self.target_timeout
        ):
            self._publish_zero_wrench(now)
            self._publish_status("target_lost")
            return
        target = self._predicted_target(now)
        if target is None:
            return
        if self.basis_u is None and not self._initialize_orbit(target):
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
        force_world = add(
            scale(position_error, self.position_kp),
            scale(subtract(desired_velocity, velocity), self.velocity_kd),
        )
        force_world = clamp_norm(force_world, self.max_force)

        current_q = self._pose_quaternion()
        desired_q = look_at_quaternion(subtract(target, position), self.plane_normal)
        angular_velocity = self._twist_angular()
        attitude_error = quaternion_error_vector(desired_q, current_q)
        torque_world = add(
            scale(attitude_error, self.attitude_kp),
            scale(angular_velocity, -self.angular_kd),
        )
        torque_world = clamp_norm(torque_world, self.max_torque)
        inverse_q = quaternion_conjugate(current_q)
        self._publish_wrench(
            now, rotate_vector(inverse_q, force_world), rotate_vector(inverse_q, torque_world)
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

    def _predicted_target(self, now: Time) -> Optional[Vector3]:
        if self.target is None or self.target_received is None:
            return None
        elapsed = max(0.0, (now - self.target_received).nanoseconds * 1.0e-9)
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

    def _publish_wrench(self, now: Time, force: Vector3, torque: Vector3) -> None:
        message = WrenchStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self.body_frame
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = force
        message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z = torque
        self.wrench_publisher.publish(message)

    def _publish_zero_wrench(self, now: Time) -> None:
        self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    def _publish_status(self, state: str, **extra: object) -> None:
        payload = {
            "state": state,
            "platform_id": self.platform_id,
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
