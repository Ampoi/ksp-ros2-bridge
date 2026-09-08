import math
import time
from typing import Any, Dict, Optional, Tuple
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import AccelStamped, PoseStamped, TransformStamped, TwistStamped, Vector3Stamped
from pylon_interfaces.msg import DockingPortCommand, DockingPortState, EngineCommand, EngineState, MotorCommand, MotorState, RcsCommand, RcsState, SeparationCommand, SeparationState, WheelCommand, WheelState, NearbyVessel, NearbyVessels
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from ..docking_packets import docking_port_manifest_from_packet, docking_port_state_from_packet
from ..motor_packets import MotorStateData, motor_state_from_packet
from ..vehicle_packets import actuator_names_to_remove, actuator_manifest_from_packet, actuator_state_from_packet, ground_truth_from_packet, nearby_vessels_from_packet
from ..domain.time_alignment import extrapolate_pose

class VehicleStateService:
    """Vehicle state adapter composed by the PyLoN ROS node."""

    def __init__(self, bridge):
        self.bridge = bridge

    def publish_ground_truth_transform_at(self, universal_time: float, stamp: Any) -> None:
        if not getattr(self.bridge, 'ground_truth_enabled', True):
            return
        state = self.bridge.latest_ground_truth
        if state is None or not math.isfinite(universal_time):
            return
        position, rotation = extrapolate_pose(
            state.position,
            state.rotation,
            state.linear_velocity,
            state.angular_velocity,
            universal_time - state.universal_time,
        )
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "pylon_ground_truth_enu"
        transform.child_frame_id = "base_link"
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ) = position
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = rotation
        self.bridge.transform_broadcaster.sendTransform(transform)

    @staticmethod
    def rotate_world_to_body(rotation: Tuple[float, float, float, float], vector: Tuple[float, float, float]) -> Tuple[float, float, float]:
        x, y, z, w = rotation
        vx, vy, vz = vector
        # Quaternion inverse rotation, expanded to avoid a geometry dependency.
        ix = w * vx - y * vz + z * vy
        iy = w * vy - z * vx + x * vz
        iz = w * vz - x * vy + y * vx
        iw = x * vx + y * vy + z * vz
        return (
            ix * w + iw * x + iy * z - iz * y,
            iy * w + iw * y + iz * x - ix * z,
            iz * w + iw * z + ix * y - iy * x,
        )

    def publish_nearby_vessels(self, packet: Dict[str, Any]) -> None:
        if not getattr(self.bridge, 'ground_truth_enabled', True):
            return
        try:
            state = nearby_vessels_from_packet(packet)
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid nearby vessel truth: {exc}")
            return
        truth = self.bridge.latest_ground_truth
        if (truth is None or state.observer_vessel_id != truth.vessel_id
                or state.origin_sequence != truth.origin_sequence):
            return
        message = NearbyVessels()
        message.header.stamp = self.bridge.flight.stamp_for_universal_time(state.universal_time)
        message.header.frame_id = "pylon_ground_truth_enu"
        message.observer_vessel_id = state.observer_vessel_id
        message.origin_sequence = state.origin_sequence
        p, v = message.observer_position, message.observer_linear_velocity
        p.x, p.y, p.z = state.observer_position
        v.x, v.y, v.z = state.observer_linear_velocity
        for target in state.vessels:
            item = NearbyVessel()
            item.vessel_id, item.vessel_name = target.vessel_id, target.vessel_name
            item.is_debris = target.is_debris
            item.position.x, item.position.y, item.position.z = target.position
            item.linear_velocity.x, item.linear_velocity.y, item.linear_velocity.z = target.linear_velocity
            message.vessels.append(item)
        self.bridge.nearby_vessels_publisher.publish(message)

    def publish_ground_truth(self, packet: Dict[str, Any]) -> None:
        if not getattr(self.bridge, 'ground_truth_enabled', True):
            return
        try:
            state = ground_truth_from_packet(packet)
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid ground truth: {exc}")
            return

        self.bridge.latest_ground_truth = state
        self.bridge.latest_ground_truth_seen_at = time.monotonic()
        stamp = self.bridge.flight.stamp_for_universal_time(state.universal_time)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "pylon_ground_truth_enu"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = state.position
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = state.rotation
        self.bridge.ground_truth_pose_publisher.publish(pose)
        if state.frame_angular_velocity is not None:
            frame_rate = Vector3Stamped(header=pose.header)
            frame_rate.vector.x, frame_rate.vector.y, frame_rate.vector.z = state.frame_angular_velocity
            self.bridge.ground_truth_frame_rate_publisher.publish(frame_rate)

        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = "pylon_ground_truth_enu"
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
        self.bridge.ground_truth_twist_publisher.publish(twist)

        body_linear = state.linear_velocity_body or self.rotate_world_to_body(
            state.rotation, state.linear_velocity
        )
        body_angular = state.angular_velocity_body or self.rotate_world_to_body(
            state.rotation, state.angular_velocity
        )
        body_twist = TwistStamped()
        body_twist.header.stamp = stamp
        body_twist.header.frame_id = "base_link"
        (
            body_twist.twist.linear.x,
            body_twist.twist.linear.y,
            body_twist.twist.linear.z,
        ) = body_linear
        (
            body_twist.twist.angular.x,
            body_twist.twist.angular.y,
            body_twist.twist.angular.z,
        ) = body_angular
        self.bridge.ground_truth_body_twist_publisher.publish(body_twist)

        acceleration = AccelStamped()
        acceleration.header.stamp = stamp
        acceleration.header.frame_id = "pylon_ground_truth_enu"
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
        self.bridge.ground_truth_acceleration_publisher.publish(acceleration)

        self.publish_ground_truth_transform_at(state.universal_time, stamp)
        self.bridge.flight.publish_vessel_lifecycle()

    def publish_actuator_state(self, packet: Dict[str, Any]) -> None:
        try:
            state = actuator_state_from_packet(packet)
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid actuator state: {exc}")
            return
        kind = state["actuatorType"]
        name = state["name"]
        publisher = self.ensure_actuator(name, kind)
        if publisher is None:
            return
        stamp = self.bridge.get_clock().now().to_msg()
        self.bridge.actuator_last_seen[name] = time.monotonic()
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
            message.header.stamp = self.bridge.flight.stamp_for_packet(packet)
            message.vessel_id = str(state.get("vesselId") or self.bridge.active_actuator_vessel_id or "")
            message.wheel_count = state["wheelCount"]
            message.radius = state["radius"]
            message.rolling_sign = state["rollingSign"]
            message.steering_sign = state["steeringSign"]
            message.steering_enabled = bool(state.get("steeringEnabled", False))
            message.max_steering_angle = state["maxSteeringAngle"]
            for field, key in (("position", "position"), ("body_min", "bodyMin"), ("body_max", "bodyMax")):
                point = getattr(message, field)
                point.x, point.y, point.z = state[key]
        elif kind == "engine":
            message = EngineState()
            message.header.stamp = stamp
            message.header.frame_id = "base_link"
            message.id = name
            message.name = name
            message.enabled = bool(state.get("enabled", False))
            message.operational = bool(state.get("operational", False))
            message.flameout = bool(state.get("flameout", False))
            message.gimbal_available = bool(state.get("gimbalAvailable", False))
            message.gimbal_command_active = bool(state.get("gimbalCommandActive", False))
            message.gimbal_pitch = state["gimbalPitch"]
            message.gimbal_yaw = state["gimbalYaw"]
            message.gimbal_roll = state["gimbalRoll"]
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
            self.bridge.separation_mechanisms[name] = message.mechanism
            message.available = bool(state.get("available", False))
            message.separated = bool(state.get("separated", False))
            if message.separated:
                self.bridge.latched_separations.add(name)
        else:
            return
        publisher.publish(message)

    def apply_actuator_manifest(self, packet: Dict[str, Any]) -> None:
        try:
            manifest = actuator_manifest_from_packet(packet)
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid actuator manifest: {exc}")
            return
        vessel_id = str(packet.get("vesselId") or "")
        vessel_changed = bool(
            self.bridge.active_actuator_vessel_id
            and vessel_id
            and vessel_id != self.bridge.active_actuator_vessel_id
        )
        for name in actuator_names_to_remove(
            manifest,
            self.bridge.actuator_kinds,
            self.bridge.latched_separations,
            vessel_changed,
        ):
            if not vessel_changed and self.bridge.actuator_kinds.get(name) == "separation":
                # KSP removes a decoupler/fairing from the active vessel in the
                # same physics transition that completes separation.  The
                # module can therefore disappear before its final state packet
                # is emitted.  Manifest removal is authoritative completion.
                self.publish_terminal_separation(name)
                continue
            self.remove_actuator(
                name,
                "active vessel changed" if vessel_changed else "not in active-vessel manifest",
            )
        if vessel_changed:
            self.reset_separation_state_publisher()
            self.bridge.latched_separations.clear()
            self.bridge.separation_mechanisms.clear()
        if vessel_id:
            self.bridge.active_actuator_vessel_id = vessel_id
        now = time.monotonic()
        for name, kind in manifest.items():
            if self.ensure_actuator(name, kind) is not None:
                self.bridge.actuator_last_seen[name] = now

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
        for kind, topic_name in self.bridge.ACTUATOR_TOPIC_NAMES.items():
            kind_state_qos = state_qos
            if kind == "separation":
                kind_state_qos = QoSProfile(depth=10)
                kind_state_qos.reliability = ReliabilityPolicy.RELIABLE
                kind_state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            base_topic = f"{self.bridge.actuators_prefix}/{topic_name}"
            self.bridge.actuator_publishers[kind] = self.bridge.create_publisher(
                state_types[kind], f"{base_topic}/state", kind_state_qos
            )
            callback = lambda message, k=kind: self.bridge.control.send_typed_actuator_command(
                k, message
            )
            self.bridge.actuator_subscriptions[kind] = self.bridge.create_subscription(
                command_types[kind], f"{base_topic}/command", callback, command_qos
            )

    def ensure_actuator(self, name: str, kind: str) -> Optional[Any]:
        previous_kind = self.bridge.actuator_kinds.get(name)
        if previous_kind is not None and previous_kind != kind:
            self.bridge.get_logger().warning(
                f"Actuator {name} changed type from {previous_kind} to {kind}; dropped"
            )
            return None
        self.bridge.actuator_kinds[name] = kind
        return self.bridge.actuator_publishers.get(kind)

    def reset_separation_state_publisher(self) -> None:
        """Drop transient-local samples that belong to the previous vessel."""
        publisher = self.bridge.actuator_publishers.pop("separation", None)
        if publisher is not None:
            self.bridge.destroy_publisher(publisher)
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        base_topic = f"{self.bridge.actuators_prefix}/{self.bridge.ACTUATOR_TOPIC_NAMES['separation']}"
        self.bridge.actuator_publishers["separation"] = self.bridge.create_publisher(
            SeparationState, f"{base_topic}/state", state_qos
        )

    def publish_terminal_separation(self, name: str) -> None:
        message = SeparationState()
        message.header.stamp = self.bridge.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.id = name
        message.name = name
        message.mechanism = self.bridge.separation_mechanisms.get(name, "decoupler")
        message.available = False
        message.separated = True
        self.bridge.latched_separations.add(name)
        self.bridge.actuator_publishers["separation"].publish(message)
        self.bridge.get_logger().info(f"Latched completed separation: {name}")

    def remove_actuator(self, name: str, reason: str) -> None:
        removed_kind = self.bridge.actuator_kinds.pop(name, None)
        self.bridge.actuator_last_seen.pop(name, None)
        self.bridge.latched_separations.discard(name)
        self.bridge.separation_mechanisms.pop(name, None)
        if removed_kind is not None:
            self.bridge.get_logger().info(f"Removed actuator ({reason}): {name}")

    def apply_docking_port_manifest(self, packet: Dict[str, Any]) -> None:
        try:
            names = docking_port_manifest_from_packet(packet)
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid docking port manifest: {exc}")
            return
        now = time.monotonic()
        active = set(names)
        for name in names:
            self.ensure_docking_port(name)
            self.bridge.docking_last_seen[name] = now
        for name in list(self.bridge.docking_publishers):
            if name not in active:
                self.remove_docking_port(name, "not in active-vessel manifest")

    def publish_docking_port_state(self, packet: Dict[str, Any]) -> None:
        try:
            state = docking_port_state_from_packet(packet)
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid docking port state: {exc}")
            return
        publisher = self.ensure_docking_port(state.name)
        if publisher is None:
            return
        message = DockingPortState()
        message.header.stamp = self.bridge.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        if self.bridge.active_model is not None:
            message.header.frame_id = self.bridge.active_model.part_frames.get(
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
        self.bridge.docking_last_seen[state.name] = time.monotonic()
        publisher.publish(message)

    def ensure_docking_port(self, name: str) -> Optional[Any]:
        existing = self.bridge.docking_publishers.get(name)
        if existing is not None:
            return existing
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        command_qos = QoSProfile(depth=10)
        command_qos.reliability = ReliabilityPolicy.RELIABLE
        base_topic = f"{self.bridge.docking_ports_prefix}/{name}"
        publisher = self.bridge.create_publisher(
            DockingPortState, f"{base_topic}/state", state_qos
        )
        callback = lambda message, n=name: self.bridge.control.send_docking_port_command(n, message)
        subscription = self.bridge.create_subscription(
            DockingPortCommand, f"{base_topic}/command", callback, command_qos
        )
        self.bridge.docking_publishers[name] = publisher
        self.bridge.docking_subscriptions[name] = subscription
        self.bridge.get_logger().info(f"Created docking port topics: {base_topic}")
        return publisher

    def remove_docking_port(self, name: str, reason: str) -> None:
        publisher = self.bridge.docking_publishers.pop(name, None)
        subscription = self.bridge.docking_subscriptions.pop(name, None)
        self.bridge.docking_last_seen.pop(name, None)
        if publisher is not None:
            self.bridge.destroy_publisher(publisher)
        if subscription is not None:
            self.bridge.destroy_subscription(subscription)
        if publisher is not None or subscription is not None:
            self.bridge.get_logger().info(f"Removed docking port ({reason}): {name}")

    def publish_motor_state(self, packet: Dict[str, Any]) -> None:
        try:
            state = motor_state_from_packet(packet)
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid motor state: {exc}")
            return

        self.bridge.motor_states[state.name] = state
        ordered_states = [self.bridge.motor_states[name] for name in sorted(self.bridge.motor_states)]
        stamp = self.bridge.get_clock().now().to_msg()

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = [state.name for state in ordered_states]
        joint_state.position = [state.position for state in ordered_states]
        joint_state.velocity = [state.velocity for state in ordered_states]
        joint_state.effort = [state.effort for state in ordered_states]
        self.bridge.joint_state_publisher.publish(joint_state)

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = stamp
        diagnostics.status = [self.motor_diagnostic(state) for state in ordered_states]
        self.bridge.diagnostics_publisher.publish(diagnostics)

        actuator_publisher = self.ensure_actuator(state.name, "motor")
        if actuator_publisher is not None:
            self.bridge.actuator_last_seen[state.name] = time.monotonic()
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
