"""LiDAR relative target estimation; target ground truth is never subscribed."""
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from .core import integrate_world_orientation, vessel_topics
from .perception import RelativeTracker, extract_clusters


class LidarPipeline:
    def __init__(self, node):
        self.node = node
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.tracker = RelativeTracker(
            measurement_sigma=node.value('measurement_sigma'),
            acceleration_sigma=node.value('acceleration_sigma'),
            max_jump=node.value('target_max_jump'), max_speed=node.value('target_max_relative_speed'),
            timeout=node.value('target_timeout_sec'))
        self.clouds = deque(maxlen=10)
        self.pose = self.twist = None
        topics = vessel_topics(node.prefix, node.value('lidar_sensor_id'), '', node.value('demo_instance_id'))
        self.subscriptions = [
            node.create_subscription(PointCloud2, node.value('lidar_topic') or topics.lidar_points, self.receive_cloud, qos_profile_sensor_data),
            node.create_subscription(PoseStamped, node.value('pose_topic') or topics.ground_truth_pose, self.receive_pose, qos_profile_sensor_data),
            node.create_subscription(TwistStamped, node.value('twist_topic') or topics.ground_truth_twist, self.receive_twist, qos_profile_sensor_data),
        ]
        self.cloud_publisher = node.create_publisher(PointCloud2, node.namespace + '/points', 10)
        self.cluster_publisher = node.create_publisher(PointCloud2, node.namespace + '/target_points', 10)
        self.timer = node.create_timer(.02, self.process)

    def reset(self):
        self.tracker.reset()
        self.clouds.clear()
        self.pose = self.twist = None

    def receive_pose(self, message):
        if message.header.frame_id == self.node.world_frame:
            self.pose = message

    def receive_twist(self, message):
        if message.header.frame_id == self.node.world_frame:
            self.twist = message

    def receive_cloud(self, message):
        if self.node.key is not None:
            self.clouds.append((message, self.node.get_clock().now()))

    def process(self):
        if not self.clouds or self.node.key is None:
            return
        message, received = self.clouds[0]
        stamp = Time.from_msg(message.header.stamp)
        timeout = self.node.value('transform_wait_timeout_sec')
        if (self.node.get_clock().now() - received).nanoseconds * 1e-9 > timeout:
            self.clouds.popleft()
            self.node.report('waiting_for_cloud_time_pose_or_mount')
            return
        if self.pose is None or self.twist is None:
            return
        pose_stamp = Time.from_msg(self.pose.header.stamp)
        if pose_stamp < stamp or Time.from_msg(self.twist.header.stamp) < stamp:
            return
        dt = (stamp - pose_stamp).nanoseconds * 1e-9
        if abs(dt) > timeout:
            self.clouds.popleft()
            return
        try:
            transform = self.buffer.lookup_transform(self.node.value('body_frame'), message.header.frame_id, Time())
        except TransformException:
            return
        self.clouds.popleft()
        q = self.pose.pose.orientation
        w = self.twist.twist.angular
        orientation = integrate_world_orientation((q.x, q.y, q.z, q.w), (w.x, w.y, w.z), dt)
        mount_q = transform.transform.rotation
        mount_p = transform.transform.translation
        try:
            points = point_cloud2.read_points_numpy(message, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
            clusters = extract_clusters(points,
                voxel_size=self.node.value('voxel_size'), tolerance=self.node.value('cluster_tolerance'),
                min_points=self.node.value('cluster_min_points'), min_range=self.node.value('min_target_range'),
                max_range=self.node.value('max_target_range'), max_points=self.node.value('max_points'),
                max_off_axis_deg=self.node.value('max_off_axis_deg'), max_extent=self.node.value('max_cluster_extent'))
            body_rotation = Rotation.from_quat(orientation)
            mount_rotation = Rotation.from_quat((mount_q.x, mount_q.y, mount_q.z, mount_q.w))
        except (ValueError, AssertionError) as error:
            self.node.report('invalid_cloud', detail=str(error))
            return
        if not clusters:
            self.tracker.observations = 0
            self.node.observations = 0
            self.node.report('no_lidar_target', points=len(points))
            return
        sensor_origin = body_rotation.apply([mount_p.x, mount_p.y, mount_p.z])
        sensor_rotation = body_rotation * mount_rotation
        centers = np.array([cluster.center for cluster in clusters])
        offset = self.node.value('target_center_offset')
        centers += centers / np.maximum(np.linalg.norm(centers, axis=1, keepdims=True), .01) * offset
        candidates = sensor_rotation.apply(centers) + sensor_origin
        result = self.tracker.update(candidates, stamp.nanoseconds * 1e-9,
                                     self.node.value('target_cluster_index'))
        if result is None:
            self.tracker.observations = 0
            self.node.observations = 0
            self.node.report('rejected_lidar_measurement')
            return
        index, position, velocity = result
        self.node.observations = self.tracker.observations
        self.node.selected_id = 'lidar_cluster'
        header = Header(stamp=message.header.stamp, frame_id=self.node.world_frame)
        self.node.publish_estimate(header, position, velocity, orientation)
        view_header = Header(stamp=message.header.stamp, frame_id=self.node.view.frame)
        finite = points[np.isfinite(points).all(axis=1)]
        self.cloud_publisher.publish(point_cloud2.create_cloud_xyz32(view_header,
            (sensor_rotation.apply(finite) + sensor_origin - position).astype(np.float32)))
        self.cluster_publisher.publish(point_cloud2.create_cloud_xyz32(view_header,
            (sensor_rotation.apply(clusters[index].points) + sensor_origin - position).astype(np.float32)))
