"""Exercise the real ROS callbacks without connecting to the live flight domain."""
import math
import unittest
from unittest.mock import Mock

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from ksp_ros2_interfaces.msg import NearbyVessels, NearbyVessel, RelativeTarget, VesselLifecycle, ControlSetpoint
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
        m = NearbyVessels(observer_vessel_id='chaser', origin_sequence=7)
        m.header.frame_id = 'ground_truth_enu'
        m.header.stamp = self.guidance.get_clock().now().to_msg()
        m.observer_position.x = 100000000.
        m.observer_linear_velocity.x = 2200.
        t = NearbyVessel(vessel_id='debris', is_debris=True)
        t.position.x = 100000015.
        t.linear_velocity.x = 2200.25
        m.vessels = [t]
        return m

    def test_truth_subtracts_same_stamp_and_locks_target(self):
        m = self.sample()
        self.estimator.receive_truth(m)
        out = self.estimator.publisher.publish.call_args.args[0]
        self.assertEqual(out.relative_position.x, 15.)
        self.assertEqual(out.relative_velocity.x, .25)
        self.assertEqual(out.header.stamp, m.header.stamp)
        m.header.stamp.nanosec += 1
        closer = NearbyVessel(vessel_id='other', is_debris=True)
        closer.position.x = 100000005.
        m.vessels = [closer]
        self.estimator.publisher.reset_mock()
        self.estimator.receive_truth(m)
        self.estimator.publisher.publish.assert_not_called()
        self.assertEqual(self.estimator.selected_id, 'debris')

    def test_relative_prediction_and_shared_orbit_setpoint(self):
        m = self.sample()
        p = PoseStamped(); p.header = m.header; p.pose.position.x = 100000000.
        p.pose.orientation.w = 1.
        v = TwistStamped(); v.header = m.header; v.twist.linear.x = 2200.
        self.guidance.receive_pose(p); self.guidance.receive_twist(v)
        out = RelativeTarget(observer_vessel_id='chaser', origin_sequence=7,
            source='truth', target_id='debris', observations=3)
        out.header = m.header; out.relative_position.x = 15.
        self.guidance.receive_target(out)
        self.guidance.control()
        command = self.guidance.setpoint_publisher.publish.call_args.args[0]
        self.assertEqual(command.mode, ControlSetpoint.MODE_SIX_DOF)
        self.assertAlmostEqual(command.position.x, p.pose.position.x)
        self.assertAlmostEqual(command.linear_velocity.x, 2200.)
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
        out.source = 'truth'; self.guidance.receive_target(out)
        self.guidance.lidar_mount_known = False
        self.guidance.control()
        self.assertEqual(self.guidance.setpoint_publisher.publish.call_args.args[0].mode,
                         ControlSetpoint.MODE_IDLE)
