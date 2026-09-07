"""Exercise the real ROS callbacks without connecting to the live flight domain."""
import math
import tempfile
from pathlib import Path
import json

import numpy as np
import unittest
from unittest.mock import Mock

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
    from std_msgs.msg import Header
    from sensor_msgs.msg import Imu, Image
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from ksp_ros2_interfaces.msg import RelativeTarget, VesselLifecycle, ControlSetpoint
    from debris_orbit.node import DebrisOrbitNode
    from debris_orbit.target_node import TargetEstimator
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'source ROS Jazzy and the built interfaces to run callback tests')
class RelativeContractTests(unittest.TestCase):
    def setUp(self):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=173)
        self.estimator = TargetEstimator(context=self.context)
        self.guidance = DebrisOrbitNode(context=self.context,
            parameter_overrides=[Parameter('enabled', value=True)])
        self.estimator.publisher = Mock()
        self.guidance.setpoint_publisher = Mock()
        self.guidance._refresh_lidar_mount = lambda: None
        self.guidance.lidar_mount_known = True
        self.epoch = VesselLifecycle(vessel_id='chaser', origin_sequence=7, state=1)
        for node in (self.estimator, self.guidance): node.receive_lifecycle(self.epoch)
        self.guidance.lidar_mount_known = True

    def tearDown(self):
        self.estimator.destroy_node()
        self.guidance.destroy_node()
        self.context.shutdown()

    def sample(self):
        m = RelativeTarget(observer_vessel_id='chaser', origin_sequence=7,
                           source='lidar_imu', target_id='lidar_cluster', observations=3)
        m.header.frame_id = 'debris_inertial'
        m.header.stamp = self.guidance.get_clock().now().to_msg()
        m.relative_position.x = 15.
        return m

    def test_lidar_and_imu_publish_navigation_without_truth_subscriptions(self):
        pipeline = self.estimator.pipeline
        mount = TransformStamped(); mount.transform.rotation.w = 1.
        pipeline.buffer.lookup_transform = Mock(return_value=mount)
        pipeline.pose_publisher = Mock(); pipeline.twist_publisher = Mock()
        for ns in (0, 100000000, 200000000):
            m = Imu(); m.header.frame_id = 'base_link'; m.header.stamp.sec = 100
            m.header.stamp.nanosec = ns
            pipeline.receive_imu(m)
        rng = np.random.default_rng(13)
        header = Header(frame_id='lidar'); header.stamp.sec = 100; header.stamp.nanosec = 100000000
        cloud = create_cloud_xyz32(header, (rng.uniform(-.4,.4,(100,3))+[15,0,0]).astype(np.float32))
        pipeline.receive_cloud(cloud); pipeline.process()
        m.header.stamp.nanosec = 300000000; pipeline.receive_imu(m)
        target = self.estimator.publisher.publish.call_args.args[0]
        pose = pipeline.pose_publisher.publish.call_args.args[0]
        self.assertAlmostEqual(target.relative_position.x, 15., delta=.1)
        self.assertAlmostEqual(pose.pose.position.x, -target.relative_position.x)
        self.assertEqual(target.source, 'lidar_imu')
        self.assertEqual(target.observations, 1)
        self.assertEqual(pose.header.stamp, target.header.stamp)
        for node in (self.estimator, self.guidance):
            subscriptions = node.get_subscriber_names_and_types_by_node(node.get_name(), node.get_namespace())
            self.assertFalse(any('/ground_truth/' in topic for topic, _ in subscriptions))

    def test_relative_prediction_and_shared_orbit_setpoint(self):
        m = self.sample()
        p = PoseStamped(); p.header = m.header; p.pose.position.x = -15.
        p.pose.orientation.w = 1.
        v = TwistStamped(); v.header = m.header; v.twist.linear.x = 0.
        self.guidance.receive_pose(p); self.guidance.receive_twist(v)
        out = RelativeTarget(observer_vessel_id='chaser', origin_sequence=7,
            source='lidar_imu', target_id='debris', observations=3)
        out.header = m.header; out.relative_position.x = 15.
        self.guidance.receive_target(out)
        self.guidance.control()
        command = self.guidance.setpoint_publisher.publish.call_args.args[0]
        self.assertEqual(command.mode, ControlSetpoint.MODE_SIX_DOF)
        self.assertAlmostEqual(command.position.x, p.pose.position.x)
        self.assertAlmostEqual(command.linear_velocity.x, 0.)
        self.assertEqual(command.header.stamp, p.header.stamp)
        # Stale/wrong epoch estimates cannot re-enable translation.
        self.epoch.origin_sequence = 8
        self.guidance.receive_lifecycle(self.epoch)
        self.guidance.receive_target(out)
        self.guidance.control()
        self.assertEqual(self.guidance.setpoint_publisher.publish.call_args.args[0].mode, ControlSetpoint.MODE_IDLE)

    def test_guidance_rejects_other_source_and_waits_for_mount(self):
        m = self.sample()
        p = PoseStamped(); p.header = m.header; p.pose.orientation.w = 1.
        v = TwistStamped(); v.header = m.header
        self.guidance.receive_pose(p); self.guidance.receive_twist(v)
        out = RelativeTarget(observer_vessel_id='chaser', origin_sequence=7,
            source='lidar', target_id='cluster', observations=3)
        out.header = m.header; out.relative_position.x = 15.
        self.guidance.receive_target(out)
        self.assertIsNone(self.guidance.target)
        out.source = 'lidar_imu'; self.guidance.receive_target(out)
        self.guidance.lidar_mount_known = False
        self.guidance.control()
        self.assertEqual(self.guidance.setpoint_publisher.publish.call_args.args[0].mode,
                         ControlSetpoint.MODE_IDLE)

    def test_camera_capture_continues_after_first_turn_and_uses_image_time(self):
        node = self.guidance
        node.basis_u, node.basis_v = (1.,0.,0.), (0.,1.,0.)
        node.actual_angle = math.radians(359.)
        node.next_capture_angle = 2*math.pi
        node.saved_captures = 10
        node.capture_sequence = 10
        node.angle_history.append((100_000_000_000, math.radians(359.)))
        p = PoseStamped(); p.header.frame_id = 'debris_inertial'
        p.header.stamp.sec = 101; p.pose.orientation.w = 1.
        node.receive_pose(p)
        node._update_capture_progress((math.cos(math.radians(1)),math.sin(math.radians(1)),0), (0,0,0))
        self.assertFalse(node.complete)
        self.assertEqual(node.pending_capture[0], 10)
        with tempfile.TemporaryDirectory() as directory:
            node.output_directory = Path(directory)
            image = Image(height=1,width=1,encoding='rgb8',step=3,data=[7,8,9])
            image.header.stamp.sec = 100; image.header.stamp.nanosec = 500000000
            image.header.frame_id = 'camera_optical'
            node.receive_image(image)
            metadata = json.loads(next(Path(directory).glob('*.json')).read_text())
            self.assertAlmostEqual(metadata['requested_angle_deg'], 360.)
            self.assertAlmostEqual(metadata['measured_angle_deg'], 360.)
            self.assertEqual(metadata['orbit'], 1)
            self.assertEqual(node.saved_captures, 11)
            self.assertEqual(len(list(Path(directory).glob('*.png'))), 1)

    def test_stale_image_is_not_mislabelled_as_current_angle(self):
        node = self.guidance
        node.pending_capture = (1, math.radians(36))
        node.angle_history.extend([(100_000_000_000, math.radians(35)),
                                   (101_000_000_000, math.radians(37))])
        image = Image(height=1,width=1,encoding='rgb8',step=3,data=[7,8,9])
        image.header.stamp.sec = 99
        node.receive_image(image)
        self.assertEqual(node.saved_captures, 0)
        self.assertIsNotNone(node.pending_capture)
