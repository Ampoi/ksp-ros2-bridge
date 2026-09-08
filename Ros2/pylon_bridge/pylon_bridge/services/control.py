from typing import Any, Dict
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from pylon_interfaces.msg import BodyWrenchCommand, ControlAuthorityCommand, ControlAuthorityState, DockingPortCommand, MotorCommand, WrenchFeedback
from ..docking_packets import encode_docking_port_command
from ..packet_conversion import sanitize_ros_name
from ..motor_packets import encode_motor_command
from ..vehicle_packets import actuator_command, body_wrench_command, control_authority_command, encode_vehicle_command
from ..domain.control import authority_state_from_packet, wrench_feedback_from_packet

class ControlService:
    """Control adapter composed by the PyLoN ROS node."""

    def __init__(self, bridge):
        self.bridge = bridge

    def send_control_authority(self, message: ControlAuthorityCommand) -> None:
        actions = {
            ControlAuthorityCommand.ACTION_ACQUIRE: "acquire",
            ControlAuthorityCommand.ACTION_RENEW: "renew",
            ControlAuthorityCommand.ACTION_RELEASE: "release",
            ControlAuthorityCommand.ACTION_EMERGENCY_STOP: "emergency_stop",
            ControlAuthorityCommand.ACTION_CLEAR_EMERGENCY_STOP: "clear_emergency_stop",
        }
        action = actions.get(message.action)
        if action is None:
            self.bridge.get_logger().warning("Dropped unsupported control authority action")
            return
        sequence = int(message.sequence)
        if sequence <= 0:
            self.bridge.get_logger().warning(
                "Dropped authority command without a positive sequence"
            )
            return
        try:
            command = control_authority_command(
                action,
                message.vessel_id,
                message.controller_id,
                message.lease_id,
                int(message.priority),
                message.lease_duration_sec,
                message.suppress_sas,
                sequence,
            )
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid control authority command: {exc}")
            return
        self.send_vehicle_packet(command, "control authority")

    def send_body_wrench_command(self, message: BodyWrenchCommand) -> None:
        if message.header.frame_id not in ("", "base_link"):
            self.bridge.get_logger().warning(
                "Dropped wrench command outside base_link frame: "
                f"{message.header.frame_id}"
            )
            return
        sequence = int(message.sequence)
        if sequence <= 0:
            self.bridge.get_logger().warning(
                "Dropped wrench command without a positive sequence"
            )
            return
        try:
            command = body_wrench_command(
                (
                    message.wrench.force.x, message.wrench.force.y, message.wrench.force.z,
                ),
                (
                    message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z,
                ),
                sequence,
                message.timeout_sec or self.bridge.args.vehicle_command_timeout_sec,
                message.vessel_id,
                message.controller_id,
                message.lease_id,
            )
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid body wrench command: {exc}")
            return
        self.send_vehicle_packet(command, "body wrench")

    def send_vehicle_packet(self, command: Dict[str, Any], label: str) -> None:
        try:
            self.bridge.transport.send(encode_vehicle_command(command), self.bridge.command_endpoint)
        except (OSError, ValueError) as exc:
            self.bridge.get_logger().warning(f"{label} UDP send failed: {exc}")

    def send_docking_port_command(
        self, name: str, message: DockingPortCommand
    ) -> None:
        sequence = int(message.sequence)
        try:
            payload = encode_docking_port_command(name, message.action, sequence, message.vessel_id, message.controller_id, message.lease_id)
            self.bridge.transport.send(payload, self.bridge.command_endpoint)
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid docking command for {name}: {exc}")
        except (OSError, ValueError) as exc:
            self.bridge.get_logger().warning(f"Docking command UDP send failed: {exc}")

    def send_typed_actuator_command(self, kind: str, message: Any) -> None:
        vessel_id = getattr(message, "vessel_id", "")
        controller_id = getattr(message, "controller_id", "")
        lease_id = getattr(message, "lease_id", "")
        raw_id = getattr(message, "id", "")
        if not isinstance(raw_id, str) or not raw_id.strip():
            self.bridge.get_logger().warning(f"Dropped {kind} command without an actuator id")
            return
        name = sanitize_ros_name(raw_id, "")
        if name == "_":
            self.bridge.get_logger().warning(f"Dropped {kind} command with an invalid actuator id")
            return
        known_kind = self.bridge.actuator_kinds.get(name)
        if known_kind is not None and known_kind != kind:
            self.bridge.get_logger().warning(
                f"Dropped {kind} command for {name}: actuator type is {known_kind}"
            )
            return
        sequence = int(getattr(message, "sequence", 0))
        if sequence <= 0:
            self.bridge.get_logger().warning(
                f"Dropped {kind} command without a positive sequence"
            )
            return
        if kind == "separation":
            try:
                command = actuator_command(
                    kind,
                    name,
                    {"separate": bool(message.separate)},
                    sequence,
                    vessel_id,
                    controller_id,
                    lease_id,
                )
            except ValueError as exc:
                self.bridge.get_logger().warning(
                    f"Dropped invalid separation command for {name}: {exc}"
                )
                return
            self.send_vehicle_packet(command, "separation command")
            return
        timeout = message.timeout_sec or self.bridge.args.vehicle_command_timeout_sec
        if kind == "motor":
            mode = {
                MotorCommand.MODE_POSITION: "position",
                MotorCommand.MODE_VELOCITY: "velocity",
                MotorCommand.MODE_EFFORT: "effort",
            }.get(message.mode)
            if mode is None:
                self.bridge.get_logger().warning(f"Dropped invalid motor mode for {name}")
                return
            command = {
                "type": "pylon_motor_command",
                "version": 1,
                "name": name,
                "vesselId": vessel_id,
                "controllerId": controller_id,
                "leaseId": lease_id,
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
                "sequence": sequence,
            }
            try:
                self.bridge.transport.send(encode_motor_command(command), self.bridge.command_endpoint)
            except (OSError, ValueError) as exc:
                self.bridge.get_logger().warning(f"Motor command UDP send failed: {exc}")
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
                brake=message.brake,
            )
        elif kind == "engine":
            values["targetThrust"] = message.target_thrust
            values.update(
                hasGimbalCommand=message.has_gimbal_command,
                gimbalPitch=message.gimbal_pitch,
                gimbalYaw=message.gimbal_yaw,
                gimbalRoll=message.gimbal_roll,
            )
        elif kind == "rcs":
            values["thrustLimit"] = message.thrust_limit
        try:
            command = actuator_command(
                kind, name, values, sequence,
                vessel_id, controller_id, lease_id,
            )
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid {kind} command for {name}: {exc}")
            return
        self.send_vehicle_packet(command, f"{kind} command")

    def publish_wrench_status(self, packet: Dict[str, Any]) -> None:
        try:
            feedback = wrench_feedback_from_packet(packet)
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid wrench status: {exc}")
            return

        message = WrenchFeedback()
        message.header.stamp = self.bridge.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.vessel_id = feedback.vessel_id
        message.controller_id = feedback.controller_id
        message.lease_id = feedback.lease_id
        message.sequence = feedback.sequence
        message.accepted = feedback.accepted
        message.reason = feedback.reason
        self.fill_wrench(message.requested, feedback.requested)
        self.fill_wrench(message.allocated, feedback.allocated)
        self.fill_wrench(message.achieved, feedback.achieved)
        self.fill_wrench(message.allocation_residual, feedback.allocation_residual)
        self.fill_wrench(message.tracking_residual, feedback.tracking_residual)
        message.saturation_ratio = feedback.saturation_ratio
        message.tracking_error_ratio = feedback.tracking_error_ratio
        message.saturated = feedback.saturated
        message.achieved_quality = feedback.achieved_quality
        self.bridge.wrench_feedback_publisher.publish(message)

        status = DiagnosticStatus()
        status.name = "KSP body wrench allocator"
        status.hardware_id = feedback.vessel_id or "active_vessel"
        status.level = (
            DiagnosticStatus.ERROR if not feedback.accepted
            else DiagnosticStatus.WARN if feedback.saturated
            else DiagnosticStatus.OK
        )
        status.message = (
            feedback.reason if not feedback.accepted
            else "allocator saturated" if feedback.saturated
            else "wrench allocated"
        )
        status.values = [
            KeyValue(key="saturation_ratio", value=str(feedback.saturation_ratio)),
            KeyValue(key="tracking_error_ratio", value=str(feedback.tracking_error_ratio)),
            KeyValue(key="achieved_quality", value=feedback.achieved_quality),
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = message.header.stamp
        diagnostics.status = [status]
        self.bridge.diagnostics_publisher.publish(diagnostics)

    @staticmethod
    def fill_wrench(message: Any, value: Any) -> None:
        message.force.x, message.force.y, message.force.z = value.force
        message.torque.x, message.torque.y, message.torque.z = value.torque

    def publish_control_authority_state(self, packet: Dict[str, Any]) -> None:
        try:
            state = authority_state_from_packet(packet)
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid control authority state: {exc}")
            return
        message = ControlAuthorityState()
        message.header.stamp = self.bridge.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.state = state.state
        message.vessel_id = state.vessel_id
        message.vessel_name = state.vessel_name
        message.controller_id = state.controller_id
        message.lease_id = state.lease_id
        message.priority = state.priority
        message.lease_remaining_sec = state.lease_remaining
        message.sas_suppressed = state.sas_suppressed
        message.emergency_stop = state.emergency_stop
        message.last_sequence = state.last_sequence
        message.reason = state.reason
        self.bridge.control_authority_state_publisher.publish(message)
