import time
from typing import Any, Dict, Optional
from pylon_interfaces.msg import VesselLifecycle
from rclpy.time import Time as RosTime

class FlightService:
    """Flight adapter composed by the PyLoN ROS node."""

    def __init__(self, bridge):
        self.bridge = bridge

    def observe_session(self, packet):
        try:
            event = self.bridge.runtime.observe_session(packet, time.monotonic())
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Invalid flight session: {exc}")
            return
        if event is not None:
            self.apply_session(event)

    def apply_session(self, event):
        self.bridge.active_vessel_id = self.bridge.session.key.vessel
        self.bridge.active_vessel_name = self.bridge.session.vessel_name
        self.bridge.vessel_generation = self.bridge.session.generation
        self.bridge.lifecycle_state = (VesselLifecycle.STATE_ACTIVE if self.bridge.session.available
                                else VesselLifecycle.STATE_UNAVAILABLE)
        self.bridge.lifecycle_reason = 'session_active' if self.bridge.session.available else 'session_unavailable'
        if event.session_changed:
            self.reset_ros_state()
        self.publish_vessel_lifecycle()

    def expire_session(self):
        if self.bridge.runtime.expire_session(time.monotonic(), self.bridge.args.topic_timeout_sec):
            self.reset_ros_state()
            self.bridge.lifecycle_state = VesselLifecycle.STATE_STALE
            self.bridge.lifecycle_reason = "session_timeout"
            self.publish_vessel_lifecycle(reason=self.bridge.lifecycle_reason)

    def reset_ros_state(self):
        self.bridge.latest_ground_truth = None
        self.bridge.latest_ground_truth_seen_at = 0.0
        self.bridge.sensors.last_imu_time = None
        self.bridge.camera.last_frame_times.clear()
        self.bridge.model.clear_active_model("flight session changed")
        self.bridge.static_sensor_transforms.clear()
        self.bridge.static_transform_broadcaster.clear()
        self.bridge.motor_states.clear()
        self.bridge.star_trackers = {}
        for topic in list(self.bridge.sensor_publishers):
            self.bridge.sensors.remove_publisher(topic, 'flight session changed')
        for name in list(self.bridge.actuator_kinds):
            self.bridge.vehicle_state.remove_actuator(name, 'flight session changed')
        for name in list(self.bridge.docking_publishers):
            self.bridge.vehicle_state.remove_docking_port(name, 'flight session changed')
        self.bridge.latched_separations.clear()
        self.bridge.vehicle_state.reset_separation_state_publisher()

    def stamp_for_packet(self, packet: Dict[str, Any]) -> Any:
        try:
            universal_time = float(packet.get("universalTime"))
        except (TypeError, ValueError):
            return self.bridge.get_clock().now().to_msg()
        return self.stamp_for_universal_time(universal_time)

    def stamp_for_universal_time(self, universal_time: float) -> Any:
        receipt = self.bridge.get_clock().now().nanoseconds
        nanoseconds = self.bridge.runtime.simulation_clock.map_nanoseconds(universal_time, receipt)
        return RosTime(nanoseconds=nanoseconds, clock_type=self.bridge.get_clock().clock_type).to_msg()

    def publish_vessel_lifecycle(
        self,
        model_ready: Optional[bool] = None,
        reason: Optional[str] = None,
    ) -> None:
        active_model_ready = bool(
            self.bridge.active_model is not None
            and self.bridge.model.active_model_is_current()
            and self.bridge.active_model.vessel_id == self.bridge.active_vessel_id
        )
        message = VesselLifecycle()
        message.header.stamp = self.bridge.get_clock().now().to_msg()
        message.header.frame_id = "pylon_ground_truth_enu" if self.bridge.ground_truth_enabled else ""
        message.state = self.bridge.lifecycle_state
        message.vessel_id = self.bridge.active_vessel_id
        message.vessel_name = self.bridge.active_vessel_name
        if self.bridge.session.key is not None:
            message.runtime_instance = self.bridge.session.key.instance
            message.runtime_epoch = self.bridge.session.key.epoch
            message.runtime_generation = self.bridge.session.key.generation
        message.generation = self.bridge.vessel_generation
        message.origin_sequence = self.bridge.vessel_generation
        message.world_frame = "pylon_ground_truth_enu" if self.bridge.ground_truth_enabled else ""
        message.body_frame = "base_link"
        message.model_ready = active_model_ready if model_ready is None else model_ready
        message.model_id = (
            self.bridge.active_model.model_id if active_model_ready else ""
        )
        message.reason = reason or self.bridge.lifecycle_reason
        self.bridge.vessel_lifecycle_publisher.publish(message)
