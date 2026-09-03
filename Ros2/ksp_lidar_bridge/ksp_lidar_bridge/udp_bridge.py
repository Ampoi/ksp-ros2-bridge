import argparse
import heapq
import ipaddress
import math
import socket
import struct
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from geometry_msgs.msg import (
    AccelStamped,
    PoseStamped,
    TransformStamped,
    Twist,
    TwistStamped,
    WrenchStamped,
)
from ksp_ros2_interfaces.msg import (
    DockingPortCommand,
    DockingPortState,
    EngineCommand,
    EngineState,
    MotorCommand,
    MotorState,
    RcsCommand,
    RcsState,
    SeparationCommand,
    SeparationState,
    WheelCommand,
    WheelState,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Float64, String
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory

from .camera_packets import (
    CameraFrame,
    CameraFrameAssembler,
    camera_optical_rotation,
    camera_topics,
)
from .docking_packets import (
    docking_port_manifest_from_packet,
    docking_port_state_from_packet,
    encode_docking_port_command,
)
from .packet_conversion import (
    as_int,
    decode_datagram,
    expired_topic_names,
    laser_scan_from_packet,
    lidar_topic_from_packet,
    packet_sensor_id,
    points_from_packet,
    sanitize_ros_name,
    sensor_pose_from_packet,
)
from .motor_packets import (
    MotorStateData,
    encode_motor_command,
    motor_commands_from_point,
    motor_state_from_packet,
)
from .propulsion_packets import (
    encode_propulsion_command,
    main_throttle_command,
    propulsion_command_from_json,
    propulsion_state_json,
    rcs_command,
)
from .vessel_model import UrdfChunkAssembler, VesselProxyModel
from .vehicle_packets import (
    actuator_names_to_remove,
    actuator_manifest_from_packet,
    actuator_command,
    actuator_state_from_packet,
    body_wrench_command,
    encode_vehicle_command,
    ground_truth_from_packet,
)


def positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return parsed


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bridge KerbalLiDAR sensors and motors to standard ROS2 topics."
    )
    parser.add_argument("--host", default="0.0.0.0", help="UDP bind host.")
    parser.add_argument("--port", type=int, default=49010, help="UDP bind port.")
    parser.add_argument(
        "--topic-prefix",
        default="/ksp_vessel",
        help="Active-vessel sensor Topic prefix.",
    )
    parser.add_argument(
        "--bridge-prefix",
        default="/ros2_ksp",
        help="Bridge-owned status and diagnostics Topic prefix.",
    )
    parser.add_argument("--frame-prefix", default="ros2_ksp")
    parser.add_argument(
        "--robot-description-topic",
        default="/ksp_vessel/robot_description",
    )
    parser.add_argument(
        "--root-frame-topic",
        default="/ksp_vessel/root_frame",
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
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--node-name", default="ksp_lidar_udp_bridge")
    parser.add_argument("--max-datagram-bytes", type=int, default=65535)
    parser.add_argument(
        "--topic-timeout-sec",
        type=positive_float,
        default=3.0,
        help="Remove a Topic after this many seconds without a scan (default: 3.0).",
    )
    parser.add_argument("--command-host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=49011)
    parser.add_argument(
        "--motor-command-topic",
        default="/ksp_vessel/actuators/servo/trajectory",
    )
    parser.add_argument("--joint-states-topic", default="/ksp_vessel/joint_states")
    parser.add_argument("--diagnostics-topic", default="/ros2_ksp/diagnostics")
    parser.add_argument(
        "--propulsion-command-topic",
        default="/ksp_vessel/actuators/propulsion/json_command",
    )
    parser.add_argument(
        "--propulsion-state-topic",
        default="/ksp_vessel/actuators/propulsion/json_state",
    )
    parser.add_argument(
        "--main-throttle-topic",
        default="/ksp_vessel/actuators/propulsion/main_throttle",
    )
    parser.add_argument(
        "--rcs-command-topic",
        default="/ksp_vessel/actuators/rcs/twist_command",
    )
    parser.add_argument(
        "--propulsion-timeout-sec",
        type=positive_float,
        default=0.5,
        help="KSP propulsion failsafe timeout for Float64/Twist commands.",
    )
    parser.add_argument("--body-wrench-topic", default="/ksp_vessel/body_wrench")
    parser.add_argument("--ground-truth-prefix", default="/ksp_vessel/ground_truth")
    parser.add_argument("--actuators-prefix", default="/ksp_vessel/actuators")
    parser.add_argument(
        "--docking-ports-prefix",
        default="",
        help="Docking port Topic prefix (default: <topic-prefix>/docking_ports).",
    )
    parser.add_argument(
        "--vehicle-command-timeout-sec",
        type=positive_float,
        default=0.5,
        help="Failsafe timeout for body wrench and typed actuator commands.",
    )
    return parser.parse_args(argv)


class KerbalLidarUdpBridge(Node):
    ACTUATOR_TOPIC_NAMES = {
        "wheel": "wheel",
        "engine": "propulsion",
        "rcs": "rcs",
        "motor": "servo",
        "separation": "separation",
    }

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(sanitize_ros_name(args.node_name, "ksp_lidar_udp_bridge"))
        self.args = args
        self.topic_prefix = "/" + args.topic_prefix.strip("/")
        self.bridge_prefix = "/" + args.bridge_prefix.strip("/")
        self.lidar_publishers: Dict[str, Any] = {}
        self.lidar_publisher_types: Dict[str, str] = {}
        self.lidar_last_seen: Dict[str, float] = {}
        self.camera_assembler = CameraFrameAssembler()
        self.actuator_publishers: Dict[str, Any] = {}
        self.actuator_subscriptions: Dict[str, Any] = {}
        self.actuator_kinds: Dict[str, str] = {}
        self.actuator_last_seen: Dict[str, float] = {}
        self.active_actuator_vessel_id = ""
        self.latched_separations = set()
        self.docking_publishers: Dict[str, Any] = {}
        self.docking_subscriptions: Dict[str, Any] = {}
        self.docking_last_seen: Dict[str, float] = {}
        model_qos = QoSProfile(depth=1)
        model_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        model_qos.reliability = ReliabilityPolicy.RELIABLE
        self.robot_description_publisher = self.create_publisher(
            String, args.robot_description_topic, model_qos
        )
        self.root_frame_publisher = self.create_publisher(
            String, args.root_frame_topic, model_qos
        )
        self.status_publisher = self.create_publisher(
            String, f"{self.bridge_prefix}/status", model_qos
        )
        self.transform_broadcaster = TransformBroadcaster(self)
        self.model_assembler = UrdfChunkAssembler()
        self.active_model: Optional[VesselProxyModel] = None
        self.active_model_seen_at = 0.0
        self.remote_model_warning_shown = False
        self.cleared_model_sessions: Dict[str, float] = {}
        self.motor_states: Dict[str, MotorStateData] = {}
        self.pending_commands: List[Tuple[int, int, Dict[str, Any]]] = []
        self.pending_command_order = 0
        self.command_sequence = 0
        self.command_endpoint = (args.command_host, args.command_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((args.host, args.port))
        self.sock.setblocking(False)
        motor_command_topic = (
            args.motor_command_topic
            or f"{self.topic_prefix}/actuators/servo/trajectory"
        )
        self.motor_command_subscription = self.create_subscription(
            JointTrajectory, motor_command_topic, self.queue_motor_trajectory, 10
        )
        self.joint_state_publisher = self.create_publisher(
            JointState, args.joint_states_topic, 10
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, args.diagnostics_topic, 10
        )
        self.propulsion_state_publisher = self.create_publisher(
            String, args.propulsion_state_topic, 10
        )
        self.propulsion_command_subscription = self.create_subscription(
            String,
            args.propulsion_command_topic,
            self.send_propulsion_json_command,
            10,
        )
        self.main_throttle_subscription = self.create_subscription(
            Float64,
            args.main_throttle_topic,
            self.send_main_throttle_command,
            10,
        )
        self.rcs_command_subscription = self.create_subscription(
            Twist,
            args.rcs_command_topic,
            self.send_rcs_command,
            10,
        )
        command_qos = QoSProfile(depth=10)
        command_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        ground_truth_prefix = "/" + args.ground_truth_prefix.strip("/")
        self.actuators_prefix = "/" + args.actuators_prefix.strip("/")
        self.docking_ports_prefix = (
            "/" + args.docking_ports_prefix.strip("/")
            if args.docking_ports_prefix.strip("/")
            else f"{self.topic_prefix}/docking_ports"
        )
        self.create_actuator_topics(command_qos, state_qos)
        self.body_wrench_subscription = self.create_subscription(
            WrenchStamped,
            args.body_wrench_topic,
            self.send_body_wrench,
            command_qos,
        )
        self.ground_truth_pose_publisher = self.create_publisher(
            PoseStamped, f"{ground_truth_prefix}/pose", state_qos
        )
        self.ground_truth_twist_publisher = self.create_publisher(
            TwistStamped, f"{ground_truth_prefix}/twist", state_qos
        )
        self.ground_truth_acceleration_publisher = self.create_publisher(
            AccelStamped, f"{ground_truth_prefix}/acceleration", state_qos
        )
        self.timer = self.create_timer(0.001, self.poll_udp)
        self.cleanup_timer = self.create_timer(0.25, self.remove_stale_publishers)
        tf_rate = max(0.5, min(float(args.model_tf_rate), 60.0))
        self.model_tf_timer = self.create_timer(1.0 / tf_rate, self.publish_model_transforms)
        self.status_publisher.publish(String(data="listening"))
        self.get_logger().info(
            f"Listening on udp://{args.host}:{args.port}; "
            f"publishing sensors under {self.topic_prefix}/<sensor_kind>/<sensor_id>; "
            f"motor commands {motor_command_topic} -> "
            f"udp://{args.command_host}:{args.command_port}"
        )
        self.get_logger().info(
            f"Active-vessel runtime proxy: {args.robot_description_topic}; "
            f"remote model packets={'allowed' if args.allow_remote_models else 'blocked'}"
        )

    def destroy_node(self) -> bool:
        self.sock.close()
        return super().destroy_node()

    def poll_udp(self) -> None:
        self.flush_motor_commands()
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
            if packet_type == "ksp_camera_inactive":
                self.remove_camera_publishers(packet, "KSP left Flight")
                continue
            if packet_type == "ksp_camera_frame_chunk":
                self.consume_camera_chunk(packet)
                continue
            if packet_type == "ksp_docking_port_manifest":
                self.apply_docking_port_manifest(packet)
                continue
            if packet_type == "ksp_lidar_scan":
                self.publish_packet(packet)
            elif packet_type == "ksp_motor_state":
                self.publish_motor_state(packet)
            elif packet_type == "ksp_propulsion_state":
                self.publish_propulsion_state(packet)
            elif packet_type == "ksp_ground_truth":
                self.publish_ground_truth(packet)
            elif packet_type == "ksp_actuator_state":
                self.publish_actuator_state(packet)
            elif packet_type == "ksp_actuator_manifest":
                self.apply_actuator_manifest(packet)
            elif packet_type == "ksp_wrench_status":
                self.publish_wrench_status(packet)
            elif packet_type == "ksp_docking_port_state":
                self.publish_docking_port_state(packet)

    def publish_packet(self, packet: Dict[str, Any]) -> None:
        mode = str(packet.get("mode", "")).upper()
        try:
            topic = lidar_topic_from_packet(packet, self.topic_prefix)
        except ValueError:
            self.get_logger().warning(f"Dropped packet with unsupported LiDAR mode: {mode}")
            return

        sensor_id = sanitize_ros_name(packet_sensor_id(packet), "lidar")
        stamp = self.get_clock().now().to_msg()
        frame_id = self.sensor_frame_for_packet(packet, sensor_id, stamp)

        if mode == "2D":
            publisher = self.publisher_for(topic, "LaserScan", LaserScan)
            if publisher is not None:
                self.lidar_last_seen[topic] = time.monotonic()
                publisher.publish(self.build_laser_scan(packet, frame_id, stamp))
            return

        if mode == "3D":
            publisher = self.publisher_for(topic, "PointCloud2", PointCloud2)
            if publisher is not None:
                self.lidar_last_seen[topic] = time.monotonic()
                publisher.publish(self.build_point_cloud(packet, frame_id, stamp))
            return

    def consume_camera_chunk(self, packet: Dict[str, Any]) -> None:
        try:
            frame = self.camera_assembler.consume(packet)
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid RGB camera packet: {exc}")
            return
        if frame is None:
            return
        self.publish_camera_frame(frame)

    def publish_camera_frame(self, frame: CameraFrame) -> None:
        image_topic, info_topic = camera_topics(
            frame.part_name,
            self.topic_prefix,
            frame.source,
            self.docking_ports_prefix,
        )
        image_publisher = self.publisher_for(image_topic, "Image", Image)
        info_publisher = self.publisher_for(info_topic, "CameraInfo", CameraInfo)
        if image_publisher is None or info_publisher is None:
            return

        stamp = self.get_clock().now().to_msg()
        optical_rotation = camera_optical_rotation(frame.frame_rotation)
        frame_id = self.sensor_frame_for_packet(
            {
                "partFlightId": frame.part_flight_id,
                "coordinateFrame": "ros_sensor",
                "framePosition": frame.frame_position,
                "frameRotation": optical_rotation,
            },
            sanitize_ros_name(
                frame.part_name,
                "docking_port" if frame.source == "docking_port" else "rgb_camera",
            ),
            stamp,
            "camera_optical_frame",
        )

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = frame_id
        image.height = frame.height
        image.width = frame.width
        image.encoding = frame.encoding
        image.is_bigendian = False
        image.step = frame.step
        image.data = frame.data

        vertical_fov = math.radians(frame.vertical_fov_degrees)
        focal_length = frame.height / (2.0 * math.tan(vertical_fov * 0.5))
        center_x = (frame.width - 1.0) * 0.5
        center_y = (frame.height - 1.0) * 0.5
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.height = frame.height
        info.width = frame.width
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [
            focal_length, 0.0, center_x,
            0.0, focal_length, center_y,
            0.0, 0.0, 1.0,
        ]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            focal_length, 0.0, center_x, 0.0,
            0.0, focal_length, center_y, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        now = time.monotonic()
        self.lidar_last_seen[image_topic] = now
        self.lidar_last_seen[info_topic] = now
        image_publisher.publish(image)
        info_publisher.publish(info)

    def consume_model_chunk(self, packet: Dict[str, Any], address: Any) -> None:
        if not self.model_source_allowed(address):
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

    def sensor_frame_for_packet(
        self,
        packet: Dict[str, Any],
        part_name: str,
        stamp: Any,
        sensor_kind: str = "lidar",
    ) -> str:
        sensor_kind = sanitize_ros_name(sensor_kind, "sensor")
        fallback = f"{sanitize_ros_name(self.args.frame_prefix)}_{part_name}_{sensor_kind}"
        part_frame = self.model_frame_for_packet(packet)
        if part_frame is None:
            return fallback

        pose = sensor_pose_from_packet(packet)
        if pose is None:
            return part_frame

        sensor_frame = f"{part_frame}_{sensor_kind}"
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
        self.transform_broadcaster.sendTransform(message)
        return sensor_frame

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
        root_transform = TransformStamped()
        root_transform.header.stamp = stamp
        root_transform.header.frame_id = "base_link"
        root_transform.child_frame_id = self.active_model.root_frame
        translation = self.active_model.base_to_root_translation
        rotation = self.active_model.base_to_root_rotation
        root_transform.transform.translation.x = translation[0]
        root_transform.transform.translation.y = translation[1]
        root_transform.transform.translation.z = translation[2]
        root_transform.transform.rotation.x = rotation[0]
        root_transform.transform.rotation.y = rotation[1]
        root_transform.transform.rotation.z = rotation[2]
        root_transform.transform.rotation.w = rotation[3]
        messages.append(root_transform)
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
            # RViz creates RELIABLE subscriptions by default. A BEST_EFFORT
            # publisher is incompatible with that request, so offer RELIABLE;
            # BEST_EFFORT subscribers can still connect to this publisher.
            publisher = self.create_publisher(message_type, topic, 10)
            self.lidar_publishers[topic] = publisher
            self.lidar_publisher_types[topic] = type_name
            self.get_logger().info(f"Created {type_name} publisher: {topic}")
        return publisher

    def remove_packet_publisher(self, packet: Dict[str, Any], reason: str) -> None:
        try:
            topics = [lidar_topic_from_packet(packet, self.topic_prefix)]
        except ValueError:
            sensor_id = sanitize_ros_name(packet_sensor_id(packet), "lidar")
            topics = [
                f"{self.topic_prefix}/lidar_2d/{sensor_id}/scan",
                f"{self.topic_prefix}/lidar_3d/{sensor_id}/points",
            ]
        for topic in topics:
            self.remove_publisher(topic, reason)

    def remove_camera_publishers(self, packet: Dict[str, Any], reason: str) -> None:
        part_name = packet_sensor_id(packet)
        source = str(packet.get("source") or "rgb_camera")
        try:
            topics = camera_topics(
                part_name, self.topic_prefix, source, self.docking_ports_prefix
            )
        except ValueError:
            return
        for topic in topics:
            self.remove_publisher(topic, reason)

    def remove_stale_publishers(self) -> None:
        self.camera_assembler.expire()
        now = time.monotonic()
        for topic in expired_topic_names(
            self.lidar_last_seen, now, self.args.topic_timeout_sec
        ):
            self.remove_publisher(topic, "scan timeout")
        for name in expired_topic_names(
            self.actuator_last_seen, now, self.args.topic_timeout_sec
        ):
            if name in self.latched_separations:
                continue
            self.remove_actuator(name, "state timeout")
        for name in expired_topic_names(
            self.docking_last_seen, now, self.args.topic_timeout_sec
        ):
            self.remove_docking_port(name, "state timeout")

    def remove_publisher(self, topic: str, reason: str) -> None:
        publisher = self.lidar_publishers.pop(topic, None)
        self.lidar_publisher_types.pop(topic, None)
        self.lidar_last_seen.pop(topic, None)
        if publisher is None:
            return

        self.destroy_publisher(publisher)
        self.get_logger().info(f"Removed publisher ({reason}): {topic}")

    def build_laser_scan(
        self, packet: Dict[str, Any], frame_id: str, stamp: Any
    ) -> LaserScan:
        data = laser_scan_from_packet(packet)
        scan = LaserScan()
        scan.header.stamp = stamp
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

    def build_point_cloud(
        self, packet: Dict[str, Any], frame_id: str, stamp: Any
    ) -> PointCloud2:
        points = points_from_packet(packet)
        payload = b"".join(struct.pack("<fff", point[0], point[1], point[2]) for point in points)

        cloud = PointCloud2()
        cloud.header.stamp = stamp
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

    def send_propulsion_json_command(self, message: String) -> None:
        self.command_sequence += 1
        try:
            command = propulsion_command_from_json(message.data, self.command_sequence)
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid propulsion command: {exc}")
            return
        self.send_propulsion_packet(command)

    def send_main_throttle_command(self, message: Float64) -> None:
        self.command_sequence += 1
        try:
            command = main_throttle_command(
                message.data,
                self.command_sequence,
                self.args.propulsion_timeout_sec,
            )
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid main throttle command: {exc}")
            return
        self.send_propulsion_packet(command)

    def send_rcs_command(self, message: Twist) -> None:
        self.command_sequence += 1
        try:
            command = rcs_command(
                message.linear.x,
                message.linear.y,
                message.linear.z,
                message.angular.x,
                message.angular.y,
                message.angular.z,
                self.command_sequence,
                self.args.propulsion_timeout_sec,
            )
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid RCS command: {exc}")
            return
        self.send_propulsion_packet(command)

    def send_propulsion_packet(self, command: Dict[str, Any]) -> None:
        try:
            self.sock.sendto(encode_propulsion_command(command), self.command_endpoint)
        except OSError as exc:
            self.get_logger().warning(f"Propulsion command UDP send failed: {exc}")

    def publish_propulsion_state(self, packet: Dict[str, Any]) -> None:
        try:
            state_json = propulsion_state_json(packet)
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid propulsion state: {exc}")
            return
        self.propulsion_state_publisher.publish(String(data=state_json))

    def send_body_wrench(self, message: WrenchStamped) -> None:
        if message.header.frame_id not in ("", "base_link"):
            self.get_logger().warning(
                "Dropped /body_wrench outside base_link frame: "
                f"{message.header.frame_id}"
            )
            return
        self.command_sequence += 1
        try:
            command = body_wrench_command(
                (
                    message.wrench.force.x,
                    message.wrench.force.y,
                    message.wrench.force.z,
                ),
                (
                    message.wrench.torque.x,
                    message.wrench.torque.y,
                    message.wrench.torque.z,
                ),
                self.command_sequence,
                self.args.vehicle_command_timeout_sec,
            )
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid body wrench: {exc}")
            return
        self.send_vehicle_packet(command, "body wrench")

    def send_vehicle_packet(self, command: Dict[str, Any], label: str) -> None:
        try:
            self.sock.sendto(encode_vehicle_command(command), self.command_endpoint)
        except OSError as exc:
            self.get_logger().warning(f"{label} UDP send failed: {exc}")

    def publish_ground_truth(self, packet: Dict[str, Any]) -> None:
        try:
            state = ground_truth_from_packet(packet)
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid ground truth: {exc}")
            return

        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "ground_truth_enu"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = state.position
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = state.rotation
        self.ground_truth_pose_publisher.publish(pose)

        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = "ground_truth_enu"
        (
            twist.twist.linear.x,
            twist.twist.linear.y,
            twist.twist.linear.z,
        ) = state.linear_velocity
        (
            twist.twist.angular.x,
            twist.twist.angular.y,
            twist.twist.angular.z,
        ) = state.angular_velocity
        self.ground_truth_twist_publisher.publish(twist)

        acceleration = AccelStamped()
        acceleration.header.stamp = stamp
        acceleration.header.frame_id = "ground_truth_enu"
        (
            acceleration.accel.linear.x,
            acceleration.accel.linear.y,
            acceleration.accel.linear.z,
        ) = state.linear_acceleration
        (
            acceleration.accel.angular.x,
            acceleration.accel.angular.y,
            acceleration.accel.angular.z,
        ) = state.angular_acceleration
        self.ground_truth_acceleration_publisher.publish(acceleration)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "ground_truth_enu"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = state.position[0]
        transform.transform.translation.y = state.position[1]
        transform.transform.translation.z = state.position[2]
        transform.transform.rotation.x = state.rotation[0]
        transform.transform.rotation.y = state.rotation[1]
        transform.transform.rotation.z = state.rotation[2]
        transform.transform.rotation.w = state.rotation[3]
        self.transform_broadcaster.sendTransform(transform)

    def publish_actuator_state(self, packet: Dict[str, Any]) -> None:
        try:
            state = actuator_state_from_packet(packet)
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid actuator state: {exc}")
            return
        kind = state["actuatorType"]
        name = state["name"]
        publisher = self.ensure_actuator(name, kind)
        if publisher is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.actuator_last_seen[name] = time.monotonic()
        if kind == "wheel":
            message = WheelState()
            message.header.stamp = stamp
            message.header.frame_id = "base_link"
            message.id = name
            message.name = name
            message.enabled = bool(state.get("enabled", False))
            message.grounded = bool(state.get("grounded", False))
            message.angular_position = state["angularPosition"]
            message.angular_velocity = state["angularVelocity"]
            message.steering_angle = state["steeringAngle"]
            message.drive_torque = state["driveTorque"]
            message.brake_torque = state["brakeTorque"]
            message.slip = state["slip"]
            message.max_drive_torque = state["maxDriveTorque"]
        elif kind == "engine":
            message = EngineState()
            message.header.stamp = stamp
            message.header.frame_id = "base_link"
            message.id = name
            message.name = name
            message.enabled = bool(state.get("enabled", False))
            message.operational = bool(state.get("operational", False))
            message.flameout = bool(state.get("flameout", False))
            message.throttle = state["throttle"]
            message.thrust = state["thrust"]
            message.max_thrust = state["maxThrust"]
            message.command_active = bool(state.get("commandActive", False))
        elif kind == "rcs":
            message = RcsState()
            message.header.stamp = stamp
            message.header.frame_id = "base_link"
            message.id = name
            message.name = name
            message.enabled = bool(state.get("enabled", False))
            message.active = bool(state.get("active", False))
            message.flameout = bool(state.get("flameout", False))
            message.thrust = state["thrust"]
            message.max_thrust = state["maxThrust"]
            message.thrust_limit = state["thrustLimit"]
            message.command_active = bool(state.get("commandActive", False))
        elif kind == "separation":
            message = SeparationState()
            message.header.stamp = stamp
            message.header.frame_id = "base_link"
            message.id = name
            message.name = name
            message.mechanism = state["mechanism"]
            message.available = bool(state.get("available", False))
            message.separated = bool(state.get("separated", False))
            if message.separated:
                self.latched_separations.add(name)
        else:
            return
        publisher.publish(message)

    def apply_actuator_manifest(self, packet: Dict[str, Any]) -> None:
        try:
            manifest = actuator_manifest_from_packet(packet)
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid actuator manifest: {exc}")
            return
        vessel_id = str(packet.get("vesselId") or "")
        vessel_changed = bool(
            self.active_actuator_vessel_id
            and vessel_id
            and vessel_id != self.active_actuator_vessel_id
        )
        for name in actuator_names_to_remove(
            manifest,
            self.actuator_kinds,
            self.latched_separations,
            vessel_changed,
        ):
            self.remove_actuator(
                name,
                "active vessel changed" if vessel_changed else "not in active-vessel manifest",
            )
        if vessel_changed:
            self.reset_separation_state_publisher()
            self.latched_separations.clear()
        if vessel_id:
            self.active_actuator_vessel_id = vessel_id
        now = time.monotonic()
        for name, kind in manifest.items():
            if self.ensure_actuator(name, kind) is not None:
                self.actuator_last_seen[name] = now

    def create_actuator_topics(self, command_qos: QoSProfile, state_qos: QoSProfile) -> None:
        state_types = {
            "wheel": WheelState,
            "engine": EngineState,
            "rcs": RcsState,
            "motor": MotorState,
            "separation": SeparationState,
        }
        command_types = {
            "wheel": WheelCommand,
            "engine": EngineCommand,
            "rcs": RcsCommand,
            "motor": MotorCommand,
            "separation": SeparationCommand,
        }
        for kind, topic_name in self.ACTUATOR_TOPIC_NAMES.items():
            kind_state_qos = state_qos
            if kind == "separation":
                kind_state_qos = QoSProfile(depth=10)
                kind_state_qos.reliability = ReliabilityPolicy.RELIABLE
                kind_state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            base_topic = f"{self.actuators_prefix}/{topic_name}"
            self.actuator_publishers[kind] = self.create_publisher(
                state_types[kind], f"{base_topic}/state", kind_state_qos
            )
            callback = lambda message, k=kind: self.send_typed_actuator_command(
                k, message
            )
            self.actuator_subscriptions[kind] = self.create_subscription(
                command_types[kind], f"{base_topic}/command", callback, command_qos
            )

    def ensure_actuator(self, name: str, kind: str) -> Optional[Any]:
        previous_kind = self.actuator_kinds.get(name)
        if previous_kind is not None and previous_kind != kind:
            self.get_logger().warning(
                f"Actuator {name} changed type from {previous_kind} to {kind}; dropped"
            )
            return None
        self.actuator_kinds[name] = kind
        return self.actuator_publishers.get(kind)

    def reset_separation_state_publisher(self) -> None:
        """Drop transient-local samples that belong to the previous vessel."""
        publisher = self.actuator_publishers.pop("separation", None)
        if publisher is not None:
            self.destroy_publisher(publisher)
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        base_topic = f"{self.actuators_prefix}/{self.ACTUATOR_TOPIC_NAMES['separation']}"
        self.actuator_publishers["separation"] = self.create_publisher(
            SeparationState, f"{base_topic}/state", state_qos
        )

    def remove_actuator(self, name: str, reason: str) -> None:
        removed_kind = self.actuator_kinds.pop(name, None)
        self.actuator_last_seen.pop(name, None)
        self.latched_separations.discard(name)
        if removed_kind is not None:
            self.get_logger().info(f"Removed actuator ({reason}): {name}")

    def apply_docking_port_manifest(self, packet: Dict[str, Any]) -> None:
        try:
            names = docking_port_manifest_from_packet(packet)
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid docking port manifest: {exc}")
            return
        now = time.monotonic()
        active = set(names)
        for name in names:
            self.ensure_docking_port(name)
            self.docking_last_seen[name] = now
        for name in list(self.docking_publishers):
            if name not in active:
                self.remove_docking_port(name, "not in active-vessel manifest")

    def publish_docking_port_state(self, packet: Dict[str, Any]) -> None:
        try:
            state = docking_port_state_from_packet(packet)
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid docking port state: {exc}")
            return
        publisher = self.ensure_docking_port(state.name)
        if publisher is None:
            return
        message = DockingPortState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        if self.active_model is not None:
            message.header.frame_id = self.active_model.part_frames.get(
                state.part_flight_id, message.header.frame_id
            )
        message.name = state.name
        message.part_flight_id = state.part_flight_id
        message.module_index = state.module_index
        message.node_type = state.node_type
        message.state = state.state
        message.docked = state.docked
        message.acquiring = state.acquiring
        message.releasable = state.releasable
        message.camera_active = state.camera_active
        message.partner_name = state.partner_name
        message.partner_part_flight_id = state.partner_part_flight_id
        self.docking_last_seen[state.name] = time.monotonic()
        publisher.publish(message)

    def ensure_docking_port(self, name: str) -> Optional[Any]:
        existing = self.docking_publishers.get(name)
        if existing is not None:
            return existing
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        command_qos = QoSProfile(depth=10)
        command_qos.reliability = ReliabilityPolicy.RELIABLE
        base_topic = f"{self.docking_ports_prefix}/{name}"
        publisher = self.create_publisher(
            DockingPortState, f"{base_topic}/state", state_qos
        )
        callback = lambda message, n=name: self.send_docking_port_command(n, message)
        subscription = self.create_subscription(
            DockingPortCommand, f"{base_topic}/command", callback, command_qos
        )
        self.docking_publishers[name] = publisher
        self.docking_subscriptions[name] = subscription
        self.get_logger().info(f"Created docking port topics: {base_topic}")
        return publisher

    def remove_docking_port(self, name: str, reason: str) -> None:
        publisher = self.docking_publishers.pop(name, None)
        subscription = self.docking_subscriptions.pop(name, None)
        self.docking_last_seen.pop(name, None)
        if publisher is not None:
            self.destroy_publisher(publisher)
        if subscription is not None:
            self.destroy_subscription(subscription)
        if publisher is not None or subscription is not None:
            self.get_logger().info(f"Removed docking port ({reason}): {name}")

    def send_docking_port_command(
        self, name: str, message: DockingPortCommand
    ) -> None:
        self.command_sequence += 1
        sequence = int(message.sequence) or self.command_sequence
        self.command_sequence = max(self.command_sequence, sequence)
        try:
            payload = encode_docking_port_command(name, message.action, sequence)
            self.sock.sendto(payload, self.command_endpoint)
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid docking command for {name}: {exc}")
        except OSError as exc:
            self.get_logger().warning(f"Docking command UDP send failed: {exc}")

    def send_typed_actuator_command(self, kind: str, message: Any) -> None:
        raw_id = getattr(message, "id", "")
        if not isinstance(raw_id, str) or not raw_id.strip():
            self.get_logger().warning(f"Dropped {kind} command without an actuator id")
            return
        name = sanitize_ros_name(raw_id, "")
        if name == "_":
            self.get_logger().warning(f"Dropped {kind} command with an invalid actuator id")
            return
        known_kind = self.actuator_kinds.get(name)
        if known_kind is not None and known_kind != kind:
            self.get_logger().warning(
                f"Dropped {kind} command for {name}: actuator type is {known_kind}"
            )
            return
        self.command_sequence += 1
        if kind == "separation":
            try:
                command = actuator_command(
                    kind,
                    name,
                    {"separate": bool(message.separate)},
                    self.command_sequence,
                )
            except ValueError as exc:
                self.get_logger().warning(
                    f"Dropped invalid separation command for {name}: {exc}"
                )
                return
            self.send_vehicle_packet(command, "separation command")
            return
        timeout = message.timeout_sec or self.args.vehicle_command_timeout_sec
        if kind == "motor":
            mode = {
                MotorCommand.MODE_POSITION: "position",
                MotorCommand.MODE_VELOCITY: "velocity",
                MotorCommand.MODE_EFFORT: "effort",
            }.get(message.mode)
            if mode is None:
                self.get_logger().warning(f"Dropped invalid motor mode for {name}")
                return
            command = {
                "type": "ksp_motor_command",
                "version": 1,
                "name": name,
                "partFlightId": 0,
                "mode": mode,
                "hasEnabled": True,
                "enabled": message.enabled,
                "hasPosition": message.enabled and mode == "position",
                "position": message.position,
                "hasVelocity": message.enabled and mode == "velocity",
                "velocity": message.velocity,
                "hasEffort": message.enabled and mode == "effort",
                "effort": message.effort,
                "timeoutSeconds": timeout,
                "sequence": self.command_sequence,
            }
            try:
                self.sock.sendto(encode_motor_command(command), self.command_endpoint)
            except OSError as exc:
                self.get_logger().warning(f"Motor command UDP send failed: {exc}")
            return

        values: Dict[str, Any] = {
            "enabled": message.enabled,
            "timeoutSeconds": timeout,
        }
        if kind == "wheel":
            values.update(
                targetAngularVelocity=message.target_angular_velocity,
                steeringAngle=message.steering_angle,
                maxDriveTorque=message.max_drive_torque,
            )
        elif kind == "engine":
            values["targetThrust"] = message.target_thrust
        elif kind == "rcs":
            values["thrustLimit"] = message.thrust_limit
        try:
            command = actuator_command(kind, name, values, self.command_sequence)
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid {kind} command for {name}: {exc}")
            return
        self.send_vehicle_packet(command, f"{kind} command")

    def publish_wrench_status(self, packet: Dict[str, Any]) -> None:
        try:
            requested_force = [float(value) for value in packet["requestedForce"]]
            requested_torque = [float(value) for value in packet["requestedTorque"]]
            residual_force = [float(value) for value in packet["residualForce"]]
            residual_torque = [float(value) for value in packet["residualTorque"]]
            if not all(
                math.isfinite(value)
                for value in requested_force + requested_torque + residual_force + residual_torque
            ):
                raise ValueError("non-finite wrench status")
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Dropped invalid wrench status: {exc}")
            return
        requested_norm = math.sqrt(sum(value * value for value in requested_force + requested_torque))
        residual_norm = math.sqrt(sum(value * value for value in residual_force + residual_torque))
        ratio = residual_norm / max(requested_norm, 1e-9)
        status = DiagnosticStatus()
        status.name = "KSP body wrench allocator"
        status.hardware_id = "active_vessel"
        status.level = DiagnosticStatus.WARN if ratio > 0.1 else DiagnosticStatus.OK
        status.message = "allocator saturated" if ratio > 0.1 else "wrench allocated"
        status.values = [
            KeyValue(key="residual_ratio", value=str(ratio)),
            KeyValue(key="residual_force", value=str(residual_force)),
            KeyValue(key="residual_torque", value=str(residual_torque)),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics_publisher.publish(message)

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

        actuator_publisher = self.ensure_actuator(state.name, "motor")
        if actuator_publisher is not None:
            self.actuator_last_seen[state.name] = time.monotonic()
            actuator = MotorState()
            actuator.header.stamp = stamp
            actuator.header.frame_id = "base_link"
            actuator.id = state.name
            actuator.name = state.name
            actuator.motor_type = state.joint_type
            actuator.enabled = state.engaged
            actuator.mode = MotorCommand.MODE_POSITION
            if state.command_mode == "velocity":
                actuator.mode = MotorCommand.MODE_VELOCITY
            elif state.command_mode == "effort":
                actuator.mode = MotorCommand.MODE_EFFORT
            actuator.position = state.position
            actuator.velocity = state.velocity
            actuator.effort = state.effort
            actuator.target = state.target
            actuator.current = state.current
            actuator.powered = state.powered
            actuator.locked = state.locked
            actuator.command_active = state.command_active
            actuator_publisher.publish(actuator)

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
