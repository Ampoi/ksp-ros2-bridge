import ipaddress
import time
from typing import Any, Dict, Optional
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from ..packet_conversion import as_int, sanitize_ros_name, sensor_pose_from_packet

class ModelService:
    """Model adapter composed by the PyLoN ROS node."""

    def __init__(self, bridge):
        self.bridge = bridge

    def consume_model_chunk(self, packet: Dict[str, Any], address: Any) -> None:
        if not self.model_source_allowed(address):
            return
        now = time.monotonic()
        self.bridge.cleared_model_sessions = {
            session_id: expires_at
            for session_id, expires_at in self.bridge.cleared_model_sessions.items()
            if expires_at > now
        }
        if packet.get("sessionId") in self.bridge.cleared_model_sessions:
            return
        try:
            model = self.bridge.model_assembler.consume(packet)
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid active-vessel URDF packet: {exc}")
            return
        if model is None:
            return

        same_model = (
            self.bridge.active_model is not None
            and self.bridge.active_model.vessel_id == model.vessel_id
            and self.bridge.active_model.model_id == model.model_id
        )
        self.bridge.active_model = model
        self.bridge.active_model_seen_at = time.monotonic()
        if same_model:
            return

        self.bridge.static_transform_broadcaster.clear()
        self.bridge.static_sensor_transforms.clear()

        self.bridge.robot_description_publisher.publish(String(data=model.urdf))
        self.bridge.root_frame_publisher.publish(String(data=model.root_frame))
        self.bridge.get_logger().info(
            f"Activated runtime vessel proxy {model.model_id[:12]}: "
            f"{len(model.part_frames)} links, root={model.root_frame}"
        )
        self.publish_model_static_transforms()
        self.publish_model_transforms()
        self.bridge.flight.publish_vessel_lifecycle(reason="model_activated")

    def consume_model_clear(self, packet: Dict[str, Any], address: Any) -> None:
        if not self.model_source_allowed(address):
            return
        session_id = packet.get("sessionId")
        if (
            packet.get("version") != 1
            or not isinstance(session_id, str)
            or len(session_id) != 32
            or any(character not in "0123456789abcdef" for character in session_id)
        ):
            return
        now = time.monotonic()
        self.bridge.cleared_model_sessions = {
            previous_session: expires_at
            for previous_session, expires_at in self.bridge.cleared_model_sessions.items()
            if expires_at > now
        }
        if len(self.bridge.cleared_model_sessions) >= 1024:
            oldest_session = min(
                self.bridge.cleared_model_sessions,
                key=self.bridge.cleared_model_sessions.get,
            )
            del self.bridge.cleared_model_sessions[oldest_session]
        self.bridge.cleared_model_sessions[session_id] = now + 120.0
        if self.bridge.active_model is not None and session_id == self.bridge.active_model.session_id:
            self.clear_active_model("KSP cleared the active vessel")

    def model_source_allowed(self, address: Any) -> bool:
        if self.bridge.args.allow_remote_models:
            return True
        try:
            source = ipaddress.ip_address(address[0])
            allowed = source.is_loopback or (
                source.version == 6
                and source.ipv4_mapped is not None
                and source.ipv4_mapped.is_loopback
            )
        except (IndexError, TypeError, ValueError):
            allowed = False
        if not allowed and not self.bridge.remote_model_warning_shown:
            self.bridge.remote_model_warning_shown = True
            self.bridge.get_logger().warning(
                f"Blocked active-vessel URDF from non-loopback source {address[0]}"
            )
        return allowed

    def model_frame_for_packet(self, packet: Dict[str, Any]) -> Optional[str]:
        if not self.active_model_is_current():
            return None
        part_id = as_int(packet.get("partFlightId"), -1)
        return self.bridge.active_model.part_frames.get(part_id)

    def sensor_frame_for_packet(
        self,
        packet: Dict[str, Any],
        part_name: str,
        stamp: Any,
        sensor_kind: str = "lidar",
        dynamic: bool = False,
    ) -> str:
        sensor_kind = sanitize_ros_name(sensor_kind, "sensor")
        sensor_frame = (
            f"{sanitize_ros_name(self.bridge.args.frame_prefix, 'pylon')}_"
            f"{sanitize_ros_name(part_name, 'sensor')}_{sensor_kind}_frame"
        )
        part_frame = self.model_frame_for_packet(packet)
        if part_frame is None:
            return sensor_frame

        pose = sensor_pose_from_packet(packet)
        if pose is None:
            return part_frame

        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id = part_frame
        message.child_frame_id = sensor_frame
        message.transform.translation.x = pose.translation[0]
        message.transform.translation.y = pose.translation[1]
        message.transform.translation.z = pose.translation[2]
        message.transform.rotation.x = pose.rotation[0]
        message.transform.rotation.y = pose.rotation[1]
        message.transform.rotation.z = pose.rotation[2]
        message.transform.rotation.w = pose.rotation[3]
        if dynamic:
            # A camera gimbal moves relative to its part. Preserve the pose at
            # each image timestamp instead of overwriting a timeless static TF.
            self.bridge.transform_broadcaster.sendTransform(message)
            return sensor_frame
        fingerprint = (
            part_frame,
            *pose.translation,
            *pose.rotation,
        )
        if self.bridge.static_sensor_transforms.get(sensor_frame) != fingerprint:
            self.bridge.static_sensor_transforms[sensor_frame] = fingerprint
            self.bridge.static_transform_broadcaster.sendTransform(message)
        return sensor_frame

    def active_model_is_current(self) -> bool:
        return self.bridge.active_model is not None and (
            time.monotonic() - self.bridge.active_model_seen_at
            <= self.bridge.active_model.expires_after_sec
        )

    def publish_model_transforms(self) -> None:
        if self.bridge.active_model is None:
            return
        if not self.active_model_is_current():
            self.clear_active_model("active-vessel runtime proxy expired")
            return

        sample = self.bridge.latest_sample_time
        stamp = self.bridge.flight.stamp_for_universal_time(sample) if sample is not None else self.bridge.get_clock().now().to_msg()
        root_transform = TransformStamped()
        root_transform.header.stamp = stamp
        root_transform.header.frame_id = "base_link"
        root_transform.child_frame_id = self.bridge.active_model.root_frame
        translation = self.bridge.active_model.base_to_root_translation
        rotation = self.bridge.active_model.base_to_root_rotation
        root_transform.transform.translation.x = translation[0]
        root_transform.transform.translation.y = translation[1]
        root_transform.transform.translation.z = translation[2]
        root_transform.transform.rotation.x = rotation[0]
        root_transform.transform.rotation.y = rotation[1]
        root_transform.transform.rotation.z = rotation[2]
        root_transform.transform.rotation.w = rotation[3]
        self.bridge.transform_broadcaster.sendTransform(root_transform)

    def publish_model_static_transforms(self) -> None:
        if self.bridge.active_model is None:
            return
        stamp = self.bridge.get_clock().now().to_msg()
        messages = []
        for transform in self.bridge.active_model.transforms:
            message = TransformStamped()
            message.header.stamp = stamp
            message.header.frame_id = transform.parent_frame
            message.child_frame_id = transform.child_frame
            message.transform.translation.x = transform.translation[0]
            message.transform.translation.y = transform.translation[1]
            message.transform.translation.z = transform.translation[2]
            message.transform.rotation.x = transform.rotation[0]
            message.transform.rotation.y = transform.rotation[1]
            message.transform.rotation.z = transform.rotation[2]
            message.transform.rotation.w = transform.rotation[3]
            messages.append(message)
        if messages:
            self.bridge.static_transform_broadcaster.sendTransform(messages)

    def clear_active_model(self, reason: str) -> None:
        if self.bridge.active_model is None:
            return
        self.bridge.active_model = None
        self.bridge.active_model_seen_at = 0.0
        self.bridge.static_transform_broadcaster.clear()
        self.bridge.static_sensor_transforms.clear()
        self.bridge.robot_description_publisher.publish(String(data=""))
        self.bridge.root_frame_publisher.publish(String(data=""))
        self.bridge.flight.publish_vessel_lifecycle(model_ready=False, reason=reason)
        self.bridge.get_logger().info(reason)
