import argparse
import json
import math
import re
import socket
import struct
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


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
    return parser.parse_args(argv)


def sanitize_ros_name(value: Any, fallback: str = "lidar") -> str:
    name = re.sub(r"[^A-Za-z0-9_]", "_", str(value or ""))
    name = re.sub(r"_+", "_", name).strip("_").lower()
    if not name:
        name = fallback
    if not re.match(r"^[A-Za-z_]", name):
        name = "_" + name
    return name


def as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def as_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def packet_lidar_name(packet: Dict[str, Any]) -> str:
    explicit = packet.get("lidarName") or packet.get("name")
    if explicit:
        return str(explicit)

    vessel = sanitize_ros_name(packet.get("vessel"), "vessel")
    part_id = as_int(packet.get("partFlightId"), 0)
    return f"{vessel}_{part_id}"


def normalized_ranges(values: Any, count: int, range_max: float) -> List[float]:
    source = values if isinstance(values, list) else []
    ranges: List[float] = []
    for index in range(count):
        raw = source[index] if index < len(source) else math.inf
        value = as_float(raw, math.inf)
        if not math.isfinite(value) or value < 0.0 or value > range_max:
            ranges.append(math.inf)
        else:
            ranges.append(value)
    return ranges


def chunked_vectors(values: Any) -> Iterable[Tuple[float, float, float]]:
    source = values if isinstance(values, list) else []
    for value in source:
        if not isinstance(value, list) or len(value) < 3:
            yield (math.nan, math.nan, math.nan)
            continue
        yield (
            as_float(value[0], math.nan),
            as_float(value[1], math.nan),
            as_float(value[2], math.nan),
        )


def points_from_packet(packet: Dict[str, Any]) -> List[Tuple[float, float, float]]:
    range_max = max(0.001, as_float(packet.get("maxDistance"), 0.001))
    ray_count = max(0, as_int(packet.get("rayCount"), 0))
    ranges = normalized_ranges(packet.get("ranges"), ray_count, range_max)

    points = packet.get("points")
    if isinstance(points, list) and points:
        output = []
        for index, point in enumerate(chunked_vectors(points)):
            if index >= len(ranges):
                break
            if math.isfinite(ranges[index]) and all(math.isfinite(axis) for axis in point):
                output.append(point)
        return output

    directions = list(chunked_vectors(packet.get("directions")))
    output = []
    for index, distance in enumerate(ranges):
        if index >= len(directions):
            break
        direction = directions[index]
        if not math.isfinite(distance) or not all(math.isfinite(axis) for axis in direction):
            continue
        output.append(
            (
                direction[0] * distance,
                direction[1] * distance,
                direction[2] * distance,
            )
        )
    return output


class KerbalLidarUdpBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(sanitize_ros_name(args.node_name, "ksp_lidar_udp_bridge"))
        self.args = args
        self.topic_prefix = "/" + args.topic_prefix.strip("/")
        self.lidar_publishers: Dict[str, Any] = {}
        self.lidar_publisher_types: Dict[str, str] = {}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((args.host, args.port))
        self.sock.setblocking(False)
        self.timer = self.create_timer(0.001, self.poll_udp)
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
                packet = json.loads(data.decode("utf-8"))
            except Exception as exc:
                self.get_logger().warning(f"Dropped invalid KerbalLiDAR packet: {exc}")
                continue

            if packet.get("type") != "ksp_lidar_scan":
                continue

            self.publish_packet(packet)

    def publish_packet(self, packet: Dict[str, Any]) -> None:
        mode = str(packet.get("mode", "")).upper()
        lidar_name = sanitize_ros_name(packet_lidar_name(packet))
        topic = f"{self.topic_prefix}/{lidar_name}"
        frame_id = f"{sanitize_ros_name(self.args.frame_prefix)}_{lidar_name}"

        if mode == "2D":
            publisher = self.publisher_for(topic, "LaserScan", LaserScan)
            if publisher is not None:
                publisher.publish(self.build_laser_scan(packet, frame_id))
            return

        if mode == "3D":
            publisher = self.publisher_for(topic, "PointCloud2", PointCloud2)
            if publisher is not None:
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

    def build_laser_scan(self, packet: Dict[str, Any], frame_id: str) -> LaserScan:
        horizontal_count = max(1, as_int(packet.get("horizontalCount"), 1))
        range_max = max(0.001, as_float(packet.get("maxDistance"), 0.001))
        fov_rad = math.radians(max(0.0, as_float(packet.get("horizontalFovDeg"), 0.0)))
        if fov_rad <= 0.0:
            fov_rad = math.tau

        angle_min = -fov_rad * 0.5
        if abs(math.degrees(fov_rad)) >= 359.9:
            angle_increment = fov_rad / horizontal_count
        else:
            angle_increment = fov_rad / max(horizontal_count - 1, 1)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = frame_id
        scan.angle_min = angle_min
        scan.angle_increment = angle_increment
        scan.angle_max = angle_min + angle_increment * max(horizontal_count - 1, 0)
        scan_rate = as_float(packet.get("scanRateHz"), 1.0)
        scan.scan_time = 1.0 / scan_rate if scan_rate > 0.0 else 0.0
        scan.time_increment = scan.scan_time / horizontal_count
        scan.range_min = 0.0
        scan.range_max = range_max
        scan.ranges = normalized_ranges(packet.get("ranges"), horizontal_count, range_max)
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
