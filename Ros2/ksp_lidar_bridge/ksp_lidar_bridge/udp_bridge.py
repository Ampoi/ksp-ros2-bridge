import argparse
import heapq
import socket
import struct
import sys
from typing import Any, Dict, List, Optional, Tuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import JointState, LaserScan, PointCloud2, PointField
from trajectory_msgs.msg import JointTrajectory

from .packet_conversion import (
    decode_datagram,
    laser_scan_from_packet,
    packet_lidar_name,
    points_from_packet,
    sanitize_ros_name,
)
from .motor_packets import (
    MotorStateData,
    encode_motor_command,
    motor_commands_from_point,
    motor_state_from_packet,
)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bridge KerbalLiDAR sensors and motors to standard ROS2 topics."
    )
    parser.add_argument("--host", default="0.0.0.0", help="UDP bind host.")
    parser.add_argument("--port", type=int, default=49010, help="UDP bind port.")
    parser.add_argument("--topic-prefix", default="/ksp_ros2/lidar")
    parser.add_argument("--frame-prefix", default="ksp_lidar")
    parser.add_argument("--node-name", default="ksp_lidar_udp_bridge")
    parser.add_argument("--max-datagram-bytes", type=int, default=65535)
    parser.add_argument("--command-host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=49011)
    parser.add_argument("--motor-command-topic", default="/ksp_ros2/motors/command")
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--diagnostics-topic", default="/diagnostics")
    return parser.parse_args(argv)


class KerbalLidarUdpBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(sanitize_ros_name(args.node_name, "ksp_lidar_udp_bridge"))
        self.args = args
        self.topic_prefix = "/" + args.topic_prefix.strip("/")
        self.lidar_publishers: Dict[str, Any] = {}
        self.lidar_publisher_types: Dict[str, str] = {}
        self.motor_states: Dict[str, MotorStateData] = {}
        self.pending_commands: List[Tuple[int, int, Dict[str, Any]]] = []
        self.pending_command_order = 0
        self.command_sequence = 0
        self.command_endpoint = (args.command_host, args.command_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((args.host, args.port))
        self.sock.setblocking(False)
        motor_command_topic = args.motor_command_topic or f"{self.topic_prefix}/motors/command"
        self.motor_command_subscription = self.create_subscription(
            JointTrajectory, motor_command_topic, self.queue_motor_trajectory, 10
        )
        self.joint_state_publisher = self.create_publisher(
            JointState, args.joint_states_topic, 10
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, args.diagnostics_topic, 10
        )
        self.timer = self.create_timer(0.001, self.poll_udp)
        self.get_logger().info(
            f"Listening on udp://{args.host}:{args.port}; "
            f"publishing LiDAR under {self.topic_prefix}/<name>; "
            f"motor commands {motor_command_topic} -> "
            f"udp://{args.command_host}:{args.command_port}"
        )

    def destroy_node(self) -> bool:
        self.sock.close()
        return super().destroy_node()

    def poll_udp(self) -> None:
        self.flush_motor_commands()
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
            elif packet_type == "ksp_motor_state":
                self.publish_motor_state(packet)

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

    def queue_motor_trajectory(self, message: JointTrajectory) -> None:
        if not message.points:
            self.get_logger().warning("Dropped motor trajectory without points")
            return

        now_ns = self.get_clock().now().nanoseconds
        self.command_sequence += 1
        sequence = self.command_sequence
        queued: List[Tuple[int, int, Dict[str, Any]]] = []
        try:
            for point in message.points:
                offset_ns = point.time_from_start.sec * 1_000_000_000
                offset_ns += point.time_from_start.nanosec
                if offset_ns < 0:
                    raise ValueError("time_from_start must not be negative")
                commands = motor_commands_from_point(
                    message.joint_names,
                    point.positions,
                    point.velocities,
                    point.effort,
                    sequence,
                )
                for command in commands:
                    self.pending_command_order += 1
                    queued.append(
                        (now_ns + offset_ns, self.pending_command_order, command)
                    )
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid motor trajectory: {exc}")
            return

        self.pending_commands.clear()
        for item in queued:
            heapq.heappush(self.pending_commands, item)
        self.flush_motor_commands()

    def flush_motor_commands(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        while self.pending_commands and self.pending_commands[0][0] <= now_ns:
            _deadline, _order, command = heapq.heappop(self.pending_commands)
            try:
                self.sock.sendto(encode_motor_command(command), self.command_endpoint)
            except OSError as exc:
                self.get_logger().warning(f"Motor command UDP send failed: {exc}")

    def publish_motor_state(self, packet: Dict[str, Any]) -> None:
        try:
            state = motor_state_from_packet(packet)
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid motor state: {exc}")
            return

        self.motor_states[state.name] = state
        ordered_states = [self.motor_states[name] for name in sorted(self.motor_states)]
        stamp = self.get_clock().now().to_msg()

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = [state.name for state in ordered_states]
        joint_state.position = [state.position for state in ordered_states]
        joint_state.velocity = [state.velocity for state in ordered_states]
        joint_state.effort = [state.effort for state in ordered_states]
        self.joint_state_publisher.publish(joint_state)

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = stamp
        diagnostics.status = [self.motor_diagnostic(state) for state in ordered_states]
        self.diagnostics_publisher.publish(diagnostics)

    @staticmethod
    def motor_diagnostic(state: MotorStateData) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = f"KSP motor/{state.name}"
        status.hardware_id = f"{state.vessel}:{state.part_flight_id}"
        if not state.powered:
            status.level = DiagnosticStatus.ERROR
            status.message = "ElectricCharge unavailable"
        elif not state.engaged:
            status.level = DiagnosticStatus.WARN
            status.message = "Motor disengaged"
        elif state.locked:
            status.level = DiagnosticStatus.WARN
            status.message = "Servo locked"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Motor operational"
        status.values = [
            KeyValue(key="joint_type", value=state.joint_type),
            KeyValue(key="position", value=str(state.position)),
            KeyValue(key="target", value=str(state.target)),
            KeyValue(key="velocity", value=str(state.velocity)),
            KeyValue(key="effort", value=str(state.effort)),
            KeyValue(key="estimated_current_a", value=str(state.current)),
        ]
        return status


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
