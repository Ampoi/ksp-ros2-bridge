import argparse
import ipaddress
import math
import os
import socket
import struct
import sys
import time
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .packet_conversion import (
    as_int,
    decode_datagram,
    expired_topic_names,
    laser_scan_from_packet,
    lidar_topic_from_packet,
    packet_part_name,
    points_from_packet,
    sanitize_ros_name,
)
from .vessel_model import UrdfChunkAssembler, VesselProxyModel


def positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return parsed


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bridge KerbalLiDAR UDP JSON packets to ROS2 sensor topics."
    )
    parser.add_argument("--host", default="0.0.0.0", help="UDP bind host.")
    parser.add_argument("--port", type=int, default=49010, help="UDP bind port.")
    parser.add_argument("--topic-prefix", default="/ros2_ksp")
    parser.add_argument("--frame-prefix", default="ros2_ksp")
    parser.add_argument(
        "--robot-description-topic",
        default="/ros2_ksp/active_vessel/robot_description",
    )
    parser.add_argument(
        "--root-frame-topic",
        default="/ros2_ksp/active_vessel/root_frame",
    )
    parser.add_argument(
        "--model-tf-rate",
        type=float,
        default=5.0,
        help="Rate used to refresh active-vessel fixed transforms on /tf.",
    )
    parser.add_argument(
        "--allow-remote-models",
        action="store_true",
        help="Accept URDF packets from non-loopback addresses (trusted networks only).",
    )
    parser.add_argument(
        "--allow-network-model-topics",
        action="store_true",
        help="Publish the runtime model without requiring ROS_LOCALHOST_ONLY=1.",
    )
    parser.add_argument("--node-name", default="ksp_lidar_udp_bridge")
    parser.add_argument("--max-datagram-bytes", type=int, default=65535)
    parser.add_argument(
        "--topic-timeout-sec",
        type=positive_float,
        default=3.0,
        help="Remove a Topic after this many seconds without a scan (default: 3.0).",
    )
    return parser.parse_args(argv)


class KerbalLidarUdpBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(sanitize_ros_name(args.node_name, "ksp_lidar_udp_bridge"))
        self.args = args
        self.topic_prefix = "/" + args.topic_prefix.strip("/")
        self.lidar_publishers: Dict[str, Any] = {}
        self.lidar_publisher_types: Dict[str, str] = {}
        self.lidar_last_seen: Dict[str, float] = {}
        model_qos = QoSProfile(depth=1)
        model_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        model_qos.reliability = ReliabilityPolicy.RELIABLE
        self.robot_description_publisher = self.create_publisher(
            String, args.robot_description_topic, model_qos
        )
        self.root_frame_publisher = self.create_publisher(
            String, args.root_frame_topic, model_qos
        )
        self.transform_broadcaster = TransformBroadcaster(self)
        self.model_assembler = UrdfChunkAssembler()
        self.active_model: Optional[VesselProxyModel] = None
        self.active_model_seen_at = 0.0
        self.remote_model_warning_shown = False
        self.cleared_model_sessions: Dict[str, float] = {}
        self.model_topics_allowed = (
            os.environ.get("ROS_LOCALHOST_ONLY") == "1"
            or args.allow_network_model_topics
        )
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((args.host, args.port))
        self.sock.setblocking(False)
        self.timer = self.create_timer(0.001, self.poll_udp)
        self.cleanup_timer = self.create_timer(0.25, self.remove_stale_publishers)
        tf_rate = max(0.5, min(float(args.model_tf_rate), 60.0))
        self.model_tf_timer = self.create_timer(1.0 / tf_rate, self.publish_model_transforms)
        self.get_logger().info(
            f"Listening on udp://{args.host}:{args.port}; "
            f"publishing under {self.topic_prefix}/<part_name>/lidar"
        )
        self.get_logger().info(
            f"Active-vessel runtime proxy: {args.robot_description_topic}; "
            f"remote model packets={'allowed' if args.allow_remote_models else 'blocked'}"
        )
        if not self.model_topics_allowed:
            self.get_logger().warning(
                "Active-vessel URDF is disabled because ROS_LOCALHOST_ONLY is not 1; "
                "restart with ROS_LOCALHOST_ONLY=1 or explicitly use "
                "--allow-network-model-topics on a protected ROS network"
            )

    def destroy_node(self) -> bool:
        self.sock.close()
        return super().destroy_node()

    def poll_udp(self) -> None:
        while True:
            try:
                data, address = self.sock.recvfrom(self.args.max_datagram_bytes)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().warning(f"UDP receive failed: {exc}")
                return

            try:
                packet = decode_datagram(data)
            except Exception as exc:
                self.get_logger().warning(f"Dropped invalid KerbalLiDAR packet: {exc}")
                continue

            packet_type = packet.get("type")
            if packet_type == "ksp_vessel_urdf_chunk":
                self.consume_model_chunk(packet, address)
                continue
            if packet_type == "ksp_vessel_urdf_clear":
                self.consume_model_clear(packet, address)
                continue
            if packet_type == "ksp_lidar_inactive":
                self.remove_packet_publisher(packet, "KSP left Flight")
                continue
            if packet_type != "ksp_lidar_scan":
                continue

            self.publish_packet(packet)

    def publish_packet(self, packet: Dict[str, Any]) -> None:
        mode = str(packet.get("mode", "")).upper()
        try:
            topic = lidar_topic_from_packet(packet, self.topic_prefix)
        except ValueError:
            self.get_logger().warning(f"Dropped packet with unsupported LiDAR mode: {mode}")
            return

        part_name = sanitize_ros_name(packet_part_name(packet), "lidar")
        frame_id = self.model_frame_for_packet(packet)
        if frame_id is None:
            frame_id = f"{sanitize_ros_name(self.args.frame_prefix)}_{part_name}_lidar"

        if mode == "2D":
            publisher = self.publisher_for(topic, "LaserScan", LaserScan)
            if publisher is not None:
                self.lidar_last_seen[topic] = time.monotonic()
                publisher.publish(self.build_laser_scan(packet, frame_id))
            return

        if mode == "3D":
            publisher = self.publisher_for(topic, "PointCloud2", PointCloud2)
            if publisher is not None:
                self.lidar_last_seen[topic] = time.monotonic()
                publisher.publish(self.build_point_cloud(packet, frame_id))
            return

    def consume_model_chunk(self, packet: Dict[str, Any], address: Any) -> None:
        if not self.model_topics_allowed or not self.model_source_allowed(address):
            return
        now = time.monotonic()
        self.cleared_model_sessions = {
            session_id: expires_at
            for session_id, expires_at in self.cleared_model_sessions.items()
            if expires_at > now
        }
        if packet.get("sessionId") in self.cleared_model_sessions:
            return
        try:
            model = self.model_assembler.consume(packet)
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid active-vessel URDF packet: {exc}")
            return
        if model is None:
            return

        same_model = (
            self.active_model is not None
            and self.active_model.session_id == model.session_id
            and self.active_model.model_id == model.model_id
        )
        self.active_model = model
        self.active_model_seen_at = time.monotonic()
        if same_model:
            return

        self.robot_description_publisher.publish(String(data=model.urdf))
        self.root_frame_publisher.publish(String(data=model.root_frame))
        self.get_logger().info(
            f"Activated runtime vessel proxy {model.model_id[:12]}: "
            f"{len(model.part_frames)} links, root={model.root_frame}"
        )
        self.publish_model_transforms()

    def consume_model_clear(self, packet: Dict[str, Any], address: Any) -> None:
        if (
            not self.model_topics_allowed
            or not self.model_source_allowed(address)
        ):
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
        self.cleared_model_sessions = {
            previous_session: expires_at
            for previous_session, expires_at in self.cleared_model_sessions.items()
            if expires_at > now
        }
        if len(self.cleared_model_sessions) >= 1024:
            oldest_session = min(
                self.cleared_model_sessions,
                key=self.cleared_model_sessions.get,
            )
            del self.cleared_model_sessions[oldest_session]
        self.cleared_model_sessions[session_id] = now + 120.0
        if self.active_model is not None and session_id == self.active_model.session_id:
            self.clear_active_model("KSP cleared the active vessel")

    def model_source_allowed(self, address: Any) -> bool:
        if self.args.allow_remote_models:
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
        if not allowed and not self.remote_model_warning_shown:
            self.remote_model_warning_shown = True
            self.get_logger().warning(
                f"Blocked active-vessel URDF from non-loopback source {address[0]}"
            )
        return allowed

    def model_frame_for_packet(self, packet: Dict[str, Any]) -> Optional[str]:
        if not self.active_model_is_current():
            return None
        part_id = as_int(packet.get("partFlightId"), -1)
        return self.active_model.part_frames.get(part_id)

    def active_model_is_current(self) -> bool:
        return self.active_model is not None and (
            time.monotonic() - self.active_model_seen_at
            <= self.active_model.expires_after_sec
        )

    def publish_model_transforms(self) -> None:
        if self.active_model is None:
            return
        if not self.active_model_is_current():
            self.clear_active_model("active-vessel runtime proxy expired")
            return

        stamp = self.get_clock().now().to_msg()
        messages = []
        for transform in self.active_model.transforms:
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
            self.transform_broadcaster.sendTransform(messages)

    def clear_active_model(self, reason: str) -> None:
        if self.active_model is None:
            return
        self.active_model = None
        self.active_model_seen_at = 0.0
        self.robot_description_publisher.publish(String(data=""))
        self.root_frame_publisher.publish(String(data=""))
        self.get_logger().info(reason)

    def publisher_for(self, topic: str, type_name: str, message_type: Any) -> Optional[Any]:
        existing_type = self.lidar_publisher_types.get(topic)
        if existing_type is not None and existing_type != type_name:
            self.get_logger().warning(
                f"Topic {topic} already uses {existing_type}; dropped {type_name} packet"
            )
            return None

        publisher = self.lidar_publishers.get(topic)
        if publisher is None:
            publisher = self.create_publisher(message_type, topic, 10)
            self.lidar_publishers[topic] = publisher
            self.lidar_publisher_types[topic] = type_name
            self.get_logger().info(f"Created {type_name} publisher: {topic}")
        return publisher

    def remove_packet_publisher(self, packet: Dict[str, Any], reason: str) -> None:
        try:
            topics = [lidar_topic_from_packet(packet, self.topic_prefix)]
        except ValueError:
            part_name = sanitize_ros_name(packet_part_name(packet), "lidar")
            base_topic = f"{self.topic_prefix}/{part_name}/lidar"
            topics = [f"{base_topic}/scan", f"{base_topic}/points"]
        for topic in topics:
            self.remove_publisher(topic, reason)

    def remove_stale_publishers(self) -> None:
        now = time.monotonic()
        for topic in expired_topic_names(
            self.lidar_last_seen, now, self.args.topic_timeout_sec
        ):
            self.remove_publisher(topic, "scan timeout")

    def remove_publisher(self, topic: str, reason: str) -> None:
        publisher = self.lidar_publishers.pop(topic, None)
        self.lidar_publisher_types.pop(topic, None)
        self.lidar_last_seen.pop(topic, None)
        if publisher is None:
            return

        self.destroy_publisher(publisher)
        self.get_logger().info(f"Removed publisher ({reason}): {topic}")

    def build_laser_scan(self, packet: Dict[str, Any], frame_id: str) -> LaserScan:
        data = laser_scan_from_packet(packet)
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = frame_id
        scan.angle_min = data.angle_min
        scan.angle_increment = data.angle_increment
        scan.angle_max = data.angle_max
        scan.scan_time = data.scan_time
        scan.time_increment = data.time_increment
        scan.range_min = data.range_min
        scan.range_max = data.range_max
        scan.ranges = data.ranges
        scan.intensities = []
        return scan

    def build_point_cloud(self, packet: Dict[str, Any], frame_id: str) -> PointCloud2:
        points = points_from_packet(packet)
        payload = b"".join(struct.pack("<fff", point[0], point[1], point[2]) for point in points)

        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = frame_id
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = payload
        cloud.is_dense = True
        return cloud


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    rclpy.init(args=None)
    node = KerbalLidarUdpBridge(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
