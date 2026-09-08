"""ROS adapter failure tests in a separate DDS domain; never send to KSP."""
import unittest
from unittest.mock import Mock
import time
try:
    import rclpy
    from pylon_demo_mun_rover.node import Rover
    from geometry_msgs.msg import Twist
except ImportError:
    rclpy=None


@unittest.skipIf(rclpy is None,'source ROS Jazzy and the built workspace')
class RuntimeTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(domain_id=178)
        self.node=Rover()
        self.node.child_goal=Mock()
        self.node.active_goal=Mock()
    def tearDown(self):
        self.node.destroy_node();rclpy.shutdown()
    def test_fault_latches_and_rejects_late_velocity(self):
        n=self.node;n.stop('imu_timeout')
        msg=Twist();msg.linear.x=.5;n.command(msg)
        self.assertEqual(n.target,(0.,0.));self.assertEqual(n.command_time,0.)
        n.child_goal.cancel_goal_async.assert_called_once()
        n.stop('second_failure');self.assertEqual(n.fault,'imu_timeout')
    def test_cancel_uses_same_stop_boundary(self):
        self.node.cancel_goal(Mock())
        self.assertEqual(self.node.fault,'goal_cancelled')
        self.node.child_goal.cancel_goal_async.assert_called_once()
    def test_nonfinite_or_holonomic_command_stops(self):
        msg=Twist();msg.linear.y=0.1;self.node.command(msg)
        self.assertEqual(self.node.fault,'unsupported_velocity')
    def test_timeout_tick_sends_brake_and_releases(self):
        n=self.node;n.ready_reason=Mock(return_value='imu_timeout');n.send_wheels=Mock();n.send_authority=Mock()
        n.tick();n.send_wheels.assert_called_once_with(0.,0.,1.)
        self.assertEqual(n.fault,'imu_timeout')
    def test_reset_refused_during_goal(self):
        from std_srvs.srv import Trigger
        response=self.node.reset_service(Trigger.Request(),Trigger.Response())
        self.assertFalse(response.success)

    def test_real_command_serialization_matches_generated_ros_types(self):
        from rclpy.serialization import serialize_message
        from pylon_vehicle_control.application.lease import LeaseAction
        from test_core import geometry
        n=self.node;n.authority_pub=Mock();n.wheel_pub=Mock()
        n.send_authority(LeaseAction('acquire','v','c','l'))
        self.assertTrue(serialize_message(n.authority_pub.publish.call_args.args[0]))
        n.geometry=geometry();n.lease.observe_vessel('v',True);n.lease.owned=True
        n.send_wheels(.2,.02,0.)
        self.assertEqual(n.wheel_pub.publish.call_count,4)
        for call in n.wheel_pub.publish.call_args_list:self.assertTrue(serialize_message(call.args[0]))
