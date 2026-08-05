import argparse
import math
import socket
import struct
import sys
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

from .packet_conversion import (
    decode_datagram,
    expired_topic_names,
    laser_scan_from_packet,
    packet_lidar_name,
    points_from_packet,
    sanitize_ros_name,
)


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
    parser.add_argument("--topic-prefix", default="/ksp_ros2/lidar")
    parser.add_argument("--frame-prefix", default="ksp_lidar")
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
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((args.host, args.port))
        self.sock.setblocking(False)
        self.timer = self.create_timer(0.001, self.poll_udp)
        self.cleanup_timer = self.create_timer(0.25, self.remove_stale_publishers)
        self.get_logger().info(
            f"Listening on udp://{args.host}:{args.port}; publishing under {self.topic_prefix}/<name>"
        )

    def destroy_node(self) -> bool:
        self.sock.close()
        return super().destroy_node()

    def poll_udp(self) -> None:
        while True:
            try:
                data, _addr = self.sock.recvfrom(self.args.max_datagram_bytes)
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
            if packet_type == "ksp_lidar_scan":
                self.publish_packet(packet)
            elif packet_type == "ksp_lidar_inactive":
                self.remove_packet_publisher(packet, "KSP left Flight")

    def publish_packet(self, packet: Dict[str, Any]) -> None:
        mode = str(packet.get("mode", "")).upper()
        lidar_name = sanitize_ros_name(packet_lidar_name(packet))
        topic = f"{self.topic_prefix}/{lidar_name}"
        frame_id = f"{sanitize_ros_name(self.args.frame_prefix)}_{lidar_name}"

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

        self.get_logger().warning(f"Dropped packet with unsupported LiDAR mode: {mode}")

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
        lidar_name = sanitize_ros_name(packet_lidar_name(packet))
        self.remove_publisher(f"{self.topic_prefix}/{lidar_name}", reason)

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
