import math
import struct
import time
from typing import Any, Dict, Optional
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from ..packet_conversion import expired_topic_names, laser_scan_from_packet, lidar_topic_from_packet, packet_sensor_id, points_from_packet, sanitize_ros_name
from ..imu_packets import imu_from_packet

class SensorsService:
    """Sensors adapter composed by the PyLoN ROS node."""

    def __init__(self, bridge):
        self.bridge = bridge
        self.last_imu_time = None

    def publish_imu(self, packet: Dict[str, Any]) -> None:
        try:
            state = imu_from_packet(packet)
        except ValueError as exc:
            self.bridge.get_logger().warning(f"Dropped invalid IMU packet: {exc}")
            return
        if self.last_imu_time is not None and state.universal_time <= self.last_imu_time:
            return
        self.last_imu_time = state.universal_time
        self.bridge.runtime.note_sample_time(state.universal_time)
        message = Imu()
        message.header.stamp = self.bridge.flight.stamp_for_packet(packet)
        message.header.frame_id = "base_link"
        # A six-axis IMU does not measure absolute orientation. Zero covariance
        # means unknown, not a claim that the simulated sensor has zero error.
        message.orientation_covariance[0] = -1.0
        (message.angular_velocity.x, message.angular_velocity.y,
         message.angular_velocity.z) = state.angular_velocity
        (message.linear_acceleration.x, message.linear_acceleration.y,
         message.linear_acceleration.z) = state.linear_acceleration
        self.bridge.imu_publisher.publish(message)

    def publish_packet(self, packet: Dict[str, Any]) -> None:
        mode = str(packet.get("mode", "")).upper()
        try:
            topic = lidar_topic_from_packet(packet, self.bridge.topic_prefix)
        except ValueError:
            self.bridge.get_logger().warning(f"Dropped packet with unsupported LiDAR mode: {mode}")
            return

        sensor_id = sanitize_ros_name(packet_sensor_id(packet), "lidar")
        stamp = self.bridge.flight.stamp_for_packet(packet)
        self.bridge.vehicle_state.publish_ground_truth_transform_at(
            float(packet.get("universalTime", math.nan)), stamp
        )
        frame_id = self.bridge.model.sensor_frame_for_packet(packet, sensor_id, stamp)

        if mode == "2D":
            publisher = self.publisher_for(topic, "LaserScan", LaserScan)
            if publisher is not None:
                self.bridge.sensor_last_seen[topic] = time.monotonic()
                publisher.publish(self.build_laser_scan(packet, frame_id, stamp))
            return

        if mode == "3D":
            publisher = self.publisher_for(topic, "PointCloud2", PointCloud2)
            if publisher is not None:
                self.bridge.sensor_last_seen[topic] = time.monotonic()
                publisher.publish(self.build_point_cloud(packet, frame_id, stamp))
            return

    def publisher_for(self, topic: str, type_name: str, message_type: Any) -> Optional[Any]:
        existing_type = self.bridge.sensor_publisher_types.get(topic)
        if existing_type is not None and existing_type != type_name:
            self.bridge.get_logger().warning(
                f"Topic {topic} already uses {existing_type}; dropped {type_name} packet"
            )
            return None

        publisher = self.bridge.sensor_publishers.get(topic)
        if publisher is None:
            # RViz creates RELIABLE subscriptions by default. A BEST_EFFORT
            # publisher is incompatible with that request, so offer RELIABLE;
            # BEST_EFFORT subscribers can still connect to this publisher.
            publisher = self.bridge.create_publisher(message_type, topic, 10)
            self.bridge.sensor_publishers[topic] = publisher
            self.bridge.sensor_publisher_types[topic] = type_name
            self.bridge.get_logger().info(f"Created {type_name} publisher: {topic}")
        return publisher

    def remove_packet_publisher(self, packet: Dict[str, Any], reason: str) -> None:
        try:
            topics = [lidar_topic_from_packet(packet, self.bridge.topic_prefix)]
        except ValueError:
            sensor_id = sanitize_ros_name(packet_sensor_id(packet), "lidar")
            topics = [
                f"{self.bridge.topic_prefix}/lidar_2d/{sensor_id}/scan",
                f"{self.bridge.topic_prefix}/lidar_3d/{sensor_id}/points",
            ]
        for topic in topics:
            self.remove_publisher(topic, reason)

    def remove_stale_publishers(self) -> None:
        now = time.monotonic()
        for topic in expired_topic_names(
            self.bridge.sensor_last_seen, now, self.bridge.args.topic_timeout_sec
        ):
            self.remove_publisher(topic, "scan timeout")
        for name in expired_topic_names(
            self.bridge.actuator_last_seen, now, self.bridge.args.topic_timeout_sec
        ):
            if name in self.bridge.latched_separations:
                continue
            self.bridge.vehicle_state.remove_actuator(name, "state timeout")
        for name in expired_topic_names(
            self.bridge.docking_last_seen, now, self.bridge.args.topic_timeout_sec
        ):
            self.bridge.vehicle_state.remove_docking_port(name, "state timeout")

    def remove_publisher(self, topic: str, reason: str) -> None:
        publisher = self.bridge.sensor_publishers.pop(topic, None)
        self.bridge.sensor_publisher_types.pop(topic, None)
        self.bridge.sensor_last_seen.pop(topic, None)
        if publisher is None:
            return

        self.bridge.destroy_publisher(publisher)
        self.bridge.get_logger().info(f"Removed publisher ({reason}): {topic}")

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
