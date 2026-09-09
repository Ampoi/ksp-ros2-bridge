import math
import time
from typing import Any, Dict
from sensor_msgs.msg import CameraInfo, Image
from ..camera_packets import CameraFrame, camera_optical_rotation, camera_topics
from ..packet_conversion import packet_sensor_id, sanitize_ros_name

class CameraService:
    """Camera adapter composed by the PyLoN ROS node."""

    def __init__(self, bridge):
        self.bridge = bridge
        self.last_frame_times = {}

    def consume_camera_chunk(self, packet: Dict[str, Any]) -> None:
        try:
            frame = self.bridge.runtime.camera_assembler.consume(packet)
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid RGB camera packet: {exc}")
            return
        if frame is None:
            return
        key = (frame.source, frame.sensor_id, frame.part_flight_id)
        if frame.universal_time <= self.last_frame_times.get(key, float('-inf')):
            return
        self.last_frame_times[key] = frame.universal_time
        self.publish_camera_frame(frame)

    def publish_camera_frame(self, frame: CameraFrame) -> None:
        image_topic, info_topic = camera_topics(
            frame.sensor_id,
            self.bridge.topic_prefix,
            frame.source,
            self.bridge.docking_ports_prefix,
        )
        image_publisher = self.bridge.sensors.publisher_for(image_topic, "Image", Image)
        info_publisher = self.bridge.sensors.publisher_for(info_topic, "CameraInfo", CameraInfo)
        if image_publisher is None or info_publisher is None:
            return

        stamp = self.bridge.flight.stamp_for_universal_time(frame.universal_time)
        self.bridge.vehicle_state.publish_ground_truth_transform_at(frame.universal_time, stamp)
        optical_rotation = camera_optical_rotation(frame.frame_rotation)
        frame_id = self.bridge.model.sensor_frame_for_packet(
            {
                "partFlightId": frame.part_flight_id,
                "coordinateFrame": "ros_sensor",
                "framePosition": frame.frame_position,
                "frameRotation": optical_rotation,
            },
            sanitize_ros_name(
                frame.sensor_id,
                "docking_port" if frame.source == "docking_port" else "rgb_camera",
            ),
            stamp,
            "camera_optical_frame",
            dynamic=True,
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
        self.bridge.sensor_last_seen[image_topic] = now
        self.bridge.sensor_last_seen[info_topic] = now
        image_publisher.publish(image)
        info_publisher.publish(info)

    def remove_camera_publishers(self, packet: Dict[str, Any], reason: str) -> None:
        part_name = packet_sensor_id(packet)
        source = str(packet.get("source") or "rgb_camera")
        try:
            topics = camera_topics(
                part_name, self.bridge.topic_prefix, source, self.bridge.docking_ports_prefix
            )
        except ValueError:
            return
        for topic in topics:
            self.bridge.sensors.remove_publisher(topic, reason)
