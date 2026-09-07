"""LiDAR + IMU relative navigation, without any Ground Truth input or TF."""
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from .core import vessel_topics
from .inertial import ImuIntegrator
from .perception import RelativeTracker, extract_clusters


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


class LidarPipeline:
    def __init__(self, node):
        self.node = node
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.imu = ImuIntegrator(max_gap=node.value('imu_max_gap_sec'))
        self.tracker = RelativeTracker(
            measurement_sigma=node.value('measurement_sigma'),
            acceleration_sigma=node.value('acceleration_sigma'),
            max_jump=node.value('target_max_jump'), max_speed=node.value('target_max_relative_speed'),
            timeout=node.value('target_timeout_sec'))
        self.clouds = deque(maxlen=10)
        self.fault = None
        topics = vessel_topics(node.prefix, node.value('lidar_sensor_id'), '', node.value('demo_instance_id'))
        self.subscriptions = [
            node.create_subscription(PointCloud2, node.value('lidar_topic') or topics.lidar_points, self.receive_cloud, qos_profile_sensor_data),
            node.create_subscription(Imu, node.value('imu_topic') or topics.imu_data, self.receive_imu, qos_profile_sensor_data),
        ]
        nav = node.namespace + '/navigation'
        self.pose_publisher = node.create_publisher(PoseStamped, nav + '/pose', 10)
        self.twist_publisher = node.create_publisher(TwistStamped, nav + '/twist', 10)
        self.body_twist_publisher = node.create_publisher(TwistStamped, nav + '/twist_body', 10)
        self.cloud_publisher = node.create_publisher(PointCloud2, node.namespace + '/points', 10)
        self.cluster_publisher = node.create_publisher(PointCloud2, node.namespace + '/target_points', 10)
        self.timer = node.create_timer(.02, self.process)

    def reset(self):
        self.imu.reset()
        self.tracker.reset()
        self.clouds.clear()
        self.fault = None

    def receive_imu(self, message):
        if self.node.key is None or self.fault or message.header.frame_id != self.node.value('body_frame'):
            return
        w, a = message.angular_velocity, message.linear_acceleration
        try:
            accepted = self.imu.push(stamp_seconds(message), (w.x,w.y,w.z), (a.x,a.y,a.z))
        except ValueError as error:
            self.fault = str(error)
            self.node.get_logger().error(self.fault)
            self.node.report('imu_restart_required', detail=self.fault)
            return
        if accepted:
            self.publish_navigation(message.header.stamp)

    def publish_navigation(self, stamp):
        sample = self.imu.samples[-1]
        position, velocity = np.zeros(3), np.zeros(3)
        tracked = (self.tracker.state is not None and
                   0 <= sample.stamp-self.tracker.stamp <= self.node.value('target_timeout_sec'))
        if tracked:
            motion = self.imu.relative_motion(self.tracker.stamp, sample.stamp)
            if motion is None:
                tracked = False
            else:
                position = self.tracker.state[:3] + self.tracker.state[3:]*(sample.stamp-self.tracker.stamp) + motion[0]
                velocity = self.tracker.state[3:] + motion[1]
        # The origin follows the unforced target. Absolute translation is
        # unobservable and irrelevant to the relative controller.
        header = Header(stamp=stamp, frame_id=self.node.world_frame)
        pose = PoseStamped(header=header)
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = -position
        q = sample.rotation.as_quat()
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        twist = TwistStamped(header=header)
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = -velocity
        twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = sample.rotation.apply(sample.gyro)
        body = TwistStamped(header=Header(stamp=stamp, frame_id=self.node.value('body_frame')))
        body.twist.linear.x, body.twist.linear.y, body.twist.linear.z = sample.rotation.inv().apply(-velocity)
        body.twist.angular.x, body.twist.angular.y, body.twist.angular.z = sample.gyro
        self.pose_publisher.publish(pose)
        self.twist_publisher.publish(twist)
        self.body_twist_publisher.publish(body)
        if tracked:
            # Count accepted LiDAR scans, never the higher-rate IMU ticks.
            self.node.observations = self.tracker.observations
            self.node.publish_estimate(header, position, velocity, tuple(q))

    def receive_cloud(self, message):
        if self.node.key is not None and not self.fault:
            self.clouds.append((message, self.node.get_clock().now()))

    def process(self):
        if not self.clouds or self.node.key is None or self.fault:
            return
        message, received = self.clouds[0]
        timeout = self.node.value('transform_wait_timeout_sec')
        if (self.node.get_clock().now() - received).nanoseconds * 1e-9 > timeout:
            self.clouds.popleft()
            self.node.report('waiting_for_cloud_time_imu_or_mount')
            return
        stamp = stamp_seconds(message)
        sample = self.imu.at(stamp)
        if sample is None:
            return
        try:
            # Only the body-to-sensor mounting chain is consulted.
            transform = self.buffer.lookup_transform(self.node.value('body_frame'), message.header.frame_id, Time())
        except TransformException:
            return
        self.clouds.popleft()
        mq, mp = transform.transform.rotation, transform.transform.translation
        try:
            points = point_cloud2.read_points_numpy(message, field_names=('x','y','z'), skip_nans=True).reshape(-1,3)
            clusters = extract_clusters(points,
                voxel_size=self.node.value('voxel_size'), tolerance=self.node.value('cluster_tolerance'),
                min_points=self.node.value('cluster_min_points'), min_range=self.node.value('min_target_range'),
                max_range=self.node.value('max_target_range'), max_points=self.node.value('max_points'),
                max_off_axis_deg=self.node.value('max_off_axis_deg'),
                max_extent=self.node.value('max_cluster_extent'))
            sensor_rotation = sample.rotation * Rotation.from_quat((mq.x,mq.y,mq.z,mq.w))
        except (ValueError, AssertionError) as error:
            self.node.report('invalid_cloud', detail=str(error))
            return
        if not clusters:
            self.tracker.observations = 0
            self.node.observations = 0
            self.node.report('no_lidar_target', points=len(points))
            return
        sensor_origin = sample.rotation.apply([mp.x,mp.y,mp.z])
        centers = np.array([cluster.center for cluster in clusters])
        centers += centers / np.maximum(np.linalg.norm(centers, axis=1, keepdims=True), .01) * self.node.value('target_center_offset')
        motion = None if self.tracker.stamp is None else self.imu.relative_motion(self.tracker.stamp, stamp)
        result = self.tracker.update(sensor_rotation.apply(centers) + sensor_origin, stamp,
                                     self.node.value('target_cluster_index'), input_delta=motion)
        if result is None:
            self.tracker.observations = 0
            self.node.observations = 0
            self.node.report('rejected_lidar_measurement')
            return
        index, position, _ = result
        view_header = Header(stamp=message.header.stamp, frame_id=self.node.view.frame)
        finite = points[np.isfinite(points).all(axis=1)]
        self.cloud_publisher.publish(point_cloud2.create_cloud_xyz32(view_header,
            (sensor_rotation.apply(finite) + sensor_origin - position).astype(np.float32)))
        self.cluster_publisher.publish(point_cloud2.create_cloud_xyz32(view_header,
            (sensor_rotation.apply(clusters[index].points) + sensor_origin - position).astype(np.float32)))
