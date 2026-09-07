"""Position-estimation half of the demo; publishes no control commands."""

import json
import math

from ksp_ros2_interfaces.msg import NearbyVessels, RelativeTarget, VesselLifecycle
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

from .core import norm, subtract, vessel_topics
from .visualization import TargetView


def xyz(value):
    return (value.x, value.y, value.z)


class TargetEstimator(Node):
    def __init__(self, **kwargs):
        super().__init__('debris_target_estimator', **kwargs)
        for name, value in {
            'target_source': 'truth', 'target_vessel_id': '',
            'demo_instance_id': 'demo_vehicle', 'vessel_topic_prefix': '/ksp_vessel',
            'target_topic': '', 'truth_topic': '', 'world_frame': 'ground_truth_enu',
            'min_target_range': 1.0, 'max_target_range': 250.0,
            'orbit_radius': 15.0, 'orbit_plane_normal': [0., 0., 1.],
            'body_frame': 'base_link', 'lidar_sensor_id': 'front_lidar',
            'lidar_topic': '', 'pose_topic': '', 'twist_topic': '',
            'voxel_size': .15, 'cluster_tolerance': 1.5, 'cluster_min_points': 5,
            'max_off_axis_deg': 35., 'max_cluster_extent': 10.,
            'max_points': 4000, 'target_cluster_index': 0, 'target_center_offset': 0.,
            'measurement_sigma': .25, 'acceleration_sigma': .5,
            'target_max_jump': 3., 'target_max_relative_speed': 5.,
            'target_timeout_sec': 1., 'transform_wait_timeout_sec': .5,
        }.items():
            self.declare_parameter(name, value)
        for name in ('min_target_range', 'max_target_range', 'orbit_radius', 'voxel_size',
                     'cluster_tolerance', 'max_cluster_extent', 'max_off_axis_deg', 'measurement_sigma', 'acceleration_sigma',
                     'target_max_jump', 'target_max_relative_speed', 'target_timeout_sec', 'transform_wait_timeout_sec'):
            if not math.isfinite(self.value(name)) or self.value(name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if (self.value('min_target_range') >= self.value('max_target_range')
                or self.value('cluster_min_points') < 1 or self.value('max_points') < 1
                or self.value('target_cluster_index') < 0):
            raise ValueError('invalid range, point count, or cluster index')
        self.source = self.value('target_source')
        self.requested_id = self.value('target_vessel_id')
        self.world_frame = self.value('world_frame')
        self.prefix = '/' + self.value('vessel_topic_prefix').strip('/')
        topics = vessel_topics(self.prefix, 'front_lidar', 'orbit_camera', self.value('demo_instance_id'))
        self.namespace = topics.demo_status.rsplit('/', 1)[0]
        self.publisher = self.create_publisher(RelativeTarget, self.value('target_topic') or self.namespace + '/target', 10)
        self.status = self.create_publisher(String, self.namespace + '/estimator_status', 10)
        self.view = TargetView(self)
        self.pipeline = None
        self.key = None
        self.selected_id = self.requested_id
        self.observations = 0
        self.last_stamp = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(VesselLifecycle, self.prefix + '/lifecycle', self.receive_lifecycle, qos)
        if self.source == 'truth':
            self.create_subscription(NearbyVessels, self.value('truth_topic') or self.prefix + '/ground_truth/nearby_vessels',
                                     self.receive_truth, qos_profile_sensor_data)
        elif self.source == 'lidar':
            from .lidar_pipeline import LidarPipeline
            self.pipeline = LidarPipeline(self)
        else:
            raise ValueError('target_source must be truth or lidar')

    def value(self, name):
        return self.get_parameter(name).value

    def receive_lifecycle(self, message):
        active = message.state in (VesselLifecycle.STATE_ACTIVE, VesselLifecycle.STATE_CHANGED)
        key = (message.vessel_id, message.origin_sequence) if active else None
        if self.key != key:
            self.selected_id = self.requested_id
            self.observations = 0
            self.last_stamp = None
            self.key = key
            self.view.reset()
            if self.pipeline is not None:
                self.pipeline.reset()

    def report(self, state, **kwargs):
        self.status.publish(String(data=json.dumps(dict(state=state, source=self.source,
            target_id=self.selected_id, observations=self.observations, **kwargs))))

    def receive_truth(self, message):
        if (self.key != (message.observer_vessel_id, message.origin_sequence)
                or message.header.frame_id != self.world_frame):
            return
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        observer_position = xyz(message.observer_position)
        observer_velocity = xyz(message.observer_linear_velocity)
        candidates = []
        for item in message.vessels:
            relative = subtract(xyz(item.position), observer_position)
            if (item.vessel_id != message.observer_vessel_id
                    and (item.is_debris or self.selected_id == item.vessel_id)
                    and self.value('min_target_range') <= norm(relative) <= self.value('max_target_range')):
                candidates.append(item)
        if not self.selected_id and candidates:
            self.selected_id = min(candidates, key=lambda item: norm(subtract(xyz(item.position), observer_position))).vessel_id
        selected = next((item for item in candidates if item.vessel_id == self.selected_id), None)
        if selected is None:
            self.observations = 0
            self.report('waiting_for_truth_target')
            return
        relative = subtract(xyz(selected.position), observer_position)
        velocity = subtract(xyz(selected.linear_velocity), observer_velocity)
        if not all(math.isfinite(x) for x in relative + velocity):
            return
        self.observations += 1
        self.publish_estimate(message.header, relative, velocity)

    def publish_estimate(self, header, relative, velocity, orientation=(0., 0., 0., 1.)):
        message = RelativeTarget()
        message.header = header
        message.observer_vessel_id, message.origin_sequence = self.key
        message.target_id = self.selected_id
        message.source = self.source
        message.relative_position.x, message.relative_position.y, message.relative_position.z = relative
        message.relative_velocity.x, message.relative_velocity.y, message.relative_velocity.z = velocity
        message.observations = self.observations
        self.publisher.publish(message)
        self.view.publish(header.stamp, relative, orientation)
        self.report('tracking', range_m=round(norm(relative), 3), relative_speed_mps=round(norm(velocity), 3))


def main(args=None):
    rclpy.init(args=args)
    node = TargetEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
