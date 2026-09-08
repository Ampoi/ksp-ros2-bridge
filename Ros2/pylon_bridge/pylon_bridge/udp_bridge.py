from .services.sensors import SensorsService
from .services.camera import CameraService
from .services.model import ModelService
from .services.flight import FlightService
from .services.control import ControlService
from .services.vehicle_state import VehicleStateService
from .services.star_tracker import StarTrackerService
import argparse
import math
import sys
from typing import Any, Dict, List, Optional, Tuple

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from geometry_msgs.msg import AccelStamped, PoseStamped, TwistStamped, Vector3Stamped
from pylon_interfaces.msg import BodyWrenchCommand, ControlAuthorityCommand, ControlAuthorityState, WrenchFeedback, VesselLifecycle, NearbyVessels
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


from .camera_packets import CameraFrameAssembler
from .packet_conversion import decode_datagram, sanitize_ros_name
from .motor_packets import MotorStateData
from .vessel_model import UrdfChunkAssembler, VesselProxyModel
from .domain.time_alignment import SimulationClock
from .static_transforms import StaticTransformSnapshot
from .domain.session import SessionTracker
from .transport import UdpTransport


def positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return parsed


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bridge PyLoN sensors and motors to standard ROS2 topics."
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
        default="/pylon",
        help="Bridge-owned status and diagnostics Topic prefix.",
    )
    parser.add_argument("--frame-prefix", default="pylon")
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
        help="Rate used to refresh the CoM-to-proxy-root transform on /tf.",
    )
    parser.add_argument(
        "--allow-remote-models",
        action="store_true",
        help="Accept URDF packets from non-loopback addresses (trusted networks only).",
    )
    parser.add_argument("--node-name", default="pylon_bridge")
    parser.add_argument("--max-datagram-bytes", type=int, default=65535)
    parser.add_argument(
        "--topic-timeout-sec",
        type=positive_float,
        default=3.0,
        help="Remove a Topic after this many seconds without a scan (default: 3.0).",
    )
    parser.add_argument("--command-host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=49011)
    parser.add_argument("--joint-states-topic", default="/ksp_vessel/joint_states")
    parser.add_argument("--diagnostics-topic", default="/pylon/diagnostics")
    parser.add_argument(
        "--body-wrench-command-topic",
        default="/ksp_vessel/control/wrench_command",
        help="Lease-bound typed body-wrench command Topic.",
    )
    parser.add_argument(
        "--control-authority-command-topic",
        default="/ksp_vessel/control/authority/command",
    )
    parser.add_argument(
        "--control-authority-state-topic",
        default="/ksp_vessel/control/authority/state",
    )
    parser.add_argument(
        "--wrench-feedback-topic",
        default="/ksp_vessel/control/wrench_feedback",
    )
    parser.add_argument(
        "--vessel-lifecycle-topic",
        default="/ksp_vessel/lifecycle",
    )
    parser.add_argument("--ground-truth-prefix", default="/ksp_vessel/ground_truth")
    parser.add_argument("--disable-ground-truth", action="store_true",
                        help="Drop truth packets and world TF; lifecycle still comes from session heartbeats.")
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


class PyLoNBridge(Node):
    ACTUATOR_TOPIC_NAMES = {
        "wheel": "wheel",
        "engine": "propulsion",
        "rcs": "rcs",
        "motor": "servo",
        "separation": "separation",
    }

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(sanitize_ros_name(args.node_name, "pylon_bridge"))
        self.args = args
        self.sensors = SensorsService(self)
        self.camera = CameraService(self)
        self.model = ModelService(self)
        self.flight = FlightService(self)
        self.control = ControlService(self)
        self.vehicle_state = VehicleStateService(self)
        self.star_tracker = StarTrackerService(self)
        self.ground_truth_enabled = not args.disable_ground_truth
        self.latest_sample_time = None
        self.topic_prefix = "/" + args.topic_prefix.strip("/")
        self.bridge_prefix = "/" + args.bridge_prefix.strip("/")
        self.sensor_publishers: Dict[str, Any] = {}
        self.sensor_publisher_types: Dict[str, str] = {}
        self.sensor_last_seen: Dict[str, float] = {}
        self.camera_assembler = CameraFrameAssembler()
        self.actuator_publishers: Dict[str, Any] = {}
        self.actuator_subscriptions: Dict[str, Any] = {}
        self.actuator_kinds: Dict[str, str] = {}
        self.actuator_last_seen: Dict[str, float] = {}
        self.active_actuator_vessel_id = ""
        self.latched_separations = set()
        self.separation_mechanisms: Dict[str, str] = {}
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
        self.static_transform_broadcaster = StaticTransformSnapshot(self)
        self.static_sensor_transforms: Dict[str, Tuple[Any, ...]] = {}
        self.model_assembler = UrdfChunkAssembler()
        self.active_model: Optional[VesselProxyModel] = None
        self.active_model_seen_at = 0.0
        self.remote_model_warning_shown = False
        self.cleared_model_sessions: Dict[str, float] = {}
        self.motor_states: Dict[str, MotorStateData] = {}
        self.pending_command_order = 0
        # IMU integration requires physical sample intervals, even when the game
        # runs slower than wall time or camera packets arrive late. Rebase only
        # at an explicit vessel/time reset, never on ordinary transport latency.
        self.simulation_clock = SimulationClock()
        self.latest_ground_truth = None
        self.latest_ground_truth_seen_at = 0.0
        self.active_vessel_id = ""
        self.active_vessel_name = ""
        self.vessel_generation = 0
        self.lifecycle_state = VesselLifecycle.STATE_UNAVAILABLE
        self.lifecycle_reason = "waiting_for_session"
        self.command_endpoint = (args.command_host, args.command_port)
        self.session = SessionTracker()
        self.transport = UdpTransport(args, self.session)
        self.sock = self.transport.socket
        self.joint_state_publisher = self.create_publisher(
            JointState, args.joint_states_topic, 10
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, args.diagnostics_topic, 10
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
        self.vehicle_state.create_actuator_topics(command_qos, state_qos)
        self.control_authority_subscription = self.create_subscription(
            ControlAuthorityCommand,
            args.control_authority_command_topic,
            self.control.send_control_authority,
            command_qos,
        )
        self.body_wrench_command_subscription = self.create_subscription(
            BodyWrenchCommand,
            args.body_wrench_command_topic,
            self.control.send_body_wrench_command,
            command_qos,
        )
        self.control_authority_state_publisher = self.create_publisher(
            ControlAuthorityState, args.control_authority_state_topic, model_qos
        )
        self.wrench_feedback_publisher = self.create_publisher(
            WrenchFeedback, args.wrench_feedback_topic, 10
        )
        self.vessel_lifecycle_publisher = self.create_publisher(
            VesselLifecycle, args.vessel_lifecycle_topic, model_qos
        )
        truth_publisher = self.create_publisher if self.ground_truth_enabled else lambda *a: None
        self.ground_truth_pose_publisher = truth_publisher(
            PoseStamped, f"{ground_truth_prefix}/pose", state_qos
        )
        self.nearby_vessels_publisher = truth_publisher(
            NearbyVessels, f"{ground_truth_prefix}/nearby_vessels", state_qos
        )
        self.ground_truth_twist_publisher = truth_publisher(
            TwistStamped, f"{ground_truth_prefix}/twist", state_qos
        )
        self.ground_truth_body_twist_publisher = truth_publisher(
            TwistStamped, f"{ground_truth_prefix}/twist_body", state_qos
        )
        self.ground_truth_acceleration_publisher = truth_publisher(
            AccelStamped, f"{ground_truth_prefix}/acceleration", state_qos
        )
        self.ground_truth_frame_rate_publisher = truth_publisher(
            Vector3Stamped, f"{ground_truth_prefix}/frame_angular_velocity", state_qos
        )
        self.imu_publisher = self.create_publisher(
            Imu, f"{self.topic_prefix}/imu/data_raw", state_qos
        )
        self.timer = self.create_timer(0.001, self.poll_udp)
        self.cleanup_timer = self.create_timer(0.25, self.sensors.remove_stale_publishers)
        tf_rate = max(0.5, min(float(args.model_tf_rate), 60.0))
        self.model_tf_timer = self.create_timer(1.0 / tf_rate, self.model.publish_model_transforms)
        self.status_publisher.publish(String(data="listening"))
        self.flight.publish_vessel_lifecycle()
        self.get_logger().info(
            f"Listening on udp://{args.host}:{args.port}; "
            f"publishing sensors under {self.topic_prefix}/<sensor_kind>/<sensor_id>; "
            f"command UDP -> "
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
        for _ in range(256):
            try:
                data, address = self.sock.recvfrom(self.args.max_datagram_bytes)
            except BlockingIOError:
                return
            except (OSError, ValueError) as exc:
                self.get_logger().warning(f"UDP receive failed: {exc}")
                return

            try:
                packet = decode_datagram(data)
            except Exception as exc:
                self.get_logger().warning(f"Dropped invalid PyLoN packet: {exc}")
                continue

            packet_type = packet.get("type")
            if packet_type == "pylon_session":
                self.flight.observe_session(packet)
                continue
            if not self.session.accepts(packet):
                continue
            if packet_type == "pylon_vessel_urdf_chunk":
                self.model.consume_model_chunk(packet, address)
                continue
            if packet_type == "pylon_vessel_urdf_clear":
                self.model.consume_model_clear(packet, address)
                continue
            if packet_type == "pylon_lidar_inactive":
                self.sensors.remove_packet_publisher(packet, "KSP left Flight")
                continue
            if packet_type == "pylon_camera_inactive":
                self.camera.remove_camera_publishers(packet, "KSP left Flight")
                continue
            if packet_type == "pylon_camera_frame_chunk":
                self.camera.consume_camera_chunk(packet)
                continue
            if packet_type == "pylon_docking_port_manifest":
                self.vehicle_state.apply_docking_port_manifest(packet)
                continue
            if packet_type == "pylon_lidar_scan":
                self.sensors.publish_packet(packet)
            elif packet_type == "pylon_motor_state":
                self.vehicle_state.publish_motor_state(packet)
            elif packet_type == "pylon_star_tracker":
                self.star_tracker.publish_star_tracker(packet)
            elif packet_type == "pylon_imu":
                self.sensors.publish_imu(packet)
            elif packet_type == "pylon_ground_truth":
                self.vehicle_state.publish_ground_truth(packet)
            elif packet_type == "pylon_nearby_vessels":
                self.vehicle_state.publish_nearby_vessels(packet)
            elif packet_type == "pylon_actuator_state":
                self.vehicle_state.publish_actuator_state(packet)
            elif packet_type == "pylon_actuator_manifest":
                self.vehicle_state.apply_actuator_manifest(packet)
            elif packet_type == "pylon_wrench_status":
                self.control.publish_wrench_status(packet)
            elif packet_type == "pylon_control_authority_state":
                self.control.publish_control_authority_state(packet)
            elif packet_type == "pylon_docking_port_state":
                self.vehicle_state.publish_docking_port_state(packet)


def main(argv: Optional[List[str]] = None) -> int:
    from rclpy.utilities import remove_ros_args
    args = parse_args(remove_ros_args(args=[sys.argv[0]] + (list(argv) if argv is not None else sys.argv[1:]))[1:])
    rclpy.init(args=None)
    node = PyLoNBridge(args)
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
