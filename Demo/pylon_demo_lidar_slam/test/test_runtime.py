"""ROS adapter tests use a separate DDS domain, without a running KSP."""
import math
import unittest
from unittest.mock import Mock, patch

try:
    import rclpy
except ImportError:
    rclpy = None

if rclpy is not None:
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan
    from pylon_interfaces.msg import VesselLifecycle, ControlAuthorityState
    from rclpy.serialization import serialize_message
    from pylon_demo_lidar_slam.planar_wrench_controller import PlanarWrenchController
    from pylon_demo_lidar_slam.laser_scan_odometry import LaserScanOdometry


@unittest.skipIf(rclpy is None, 'requires ROS2 Jazzy and built pylon_interfaces')
class RuntimeTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(domain_id=179)
        self.controller = PlanarWrenchController()
        self.odometry = LaserScanOdometry()
        self.controller.publisher = Mock()
        self.controller.authority_publisher = Mock()
        self.odometry.odom_publisher = Mock()
        self.odometry.scan_publisher = Mock()
        self.odometry.transform_broadcaster = Mock()
        self.lifecycle = VesselLifecycle()
        self.lifecycle.vessel_id = 'vessel'
        self.lifecycle.generation = 1
        self.lifecycle.state = VesselLifecycle.STATE_ACTIVE
        self.controller.receive_lifecycle(self.lifecycle)
        self.odometry.receive_lifecycle(self.lifecycle)

    def tearDown(self):
        self.controller.destroy_node()
        self.odometry.destroy_node()
        rclpy.shutdown()

    def scan(self, stamp):
        scan = LaserScan()
        scan.header.stamp.sec = stamp
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / 100
        scan.range_min = 0.1
        scan.range_max = 20.0
        scan.ranges = [5.0] * 100
        return scan

    def test_generation_change_stops_and_rejects_new_commands(self):
        n = self.controller
        n.lease.owned = True
        self.lifecycle.generation = 2
        n.receive_lifecycle(self.lifecycle)
        cmd = Twist()
        cmd.linear.x = 0.5
        n.receive_command(cmd)
        n.control()
        self.assertIsNone(n.last_command)
        self.assertFalse(n.lease.owned)
        self.assertEqual(n.publisher.publish.call_args.args[0].wrench.force.x, 0.0)
        self.assertEqual(n.authority_publisher.publish.call_args.args[0].action, 3)

    def test_scan_matching_failure_does_not_publish_healthy_odometry_or_scan(self):
        n = self.odometry
        n.receive_scan(self.scan(1))
        n.odom_publisher.reset_mock()
        n.scan_publisher.reset_mock()
        with patch('pylon_demo_lidar_slam.laser_scan_odometry.iterative_closest_point',
                   side_effect=ValueError('no overlap')):
            n.receive_scan(self.scan(2))
        n.odom_publisher.publish.assert_not_called()
        n.scan_publisher.publish.assert_not_called()
        self.assertEqual(n.previous_stamp, 1.0)

    def test_insufficient_points_dont_start_mapping(self):
        scan = self.scan(1)
        scan.ranges = [math.inf] * 100
        self.odometry.receive_scan(scan)
        self.odometry.odom_publisher.publish.assert_not_called()
        self.odometry.scan_publisher.publish.assert_not_called()

    def test_scan_time_rewind_latches_until_restart(self):
        n = self.odometry
        n.receive_scan(self.scan(10))
        n.odom_publisher.reset_mock()
        n.receive_scan(self.scan(9))
        n.receive_scan(self.scan(11))
        self.assertEqual(n.session.fault, 'scan_time_discontinuity')
        n.odom_publisher.publish.assert_not_called()

    def test_odometry_timeout_blocks_future_nav2_commands(self):
        n = self.controller
        n.lease.owned = True
        n.receive_command(Twist())
        n.control()
        self.assertEqual(n.session.fault, 'odometry_timeout')
        n.receive_command(Twist())
        self.assertIsNone(n.last_command)

    def test_lost_authority_does_not_reacquire(self):
        n = self.controller
        n.lease.owned = True
        n.receive_authority(ControlAuthorityState())
        n.receive_command(Twist())
        n.control()
        self.assertEqual(n.session.fault, 'authority_lost')
        n.authority_publisher.publish.assert_not_called()

    def test_authority_and_wrench_serialize_with_current_interfaces(self):
        n = self.controller
        now = n.get_clock().now()
        n._maintain_authority(now)
        authority = n.authority_publisher.publish.call_args.args[0]
        self.assertTrue(serialize_message(authority))
        n.lease.owned = True
        n._publish_wrench(now, (1.0, 0.0, 0.0), (0.0, 0.0, 0.5))
        wrench = n.publisher.publish.call_args.args[0]
        self.assertTrue(serialize_message(wrench))
        self.assertEqual(wrench.vessel_id, authority.vessel_id)
        self.assertEqual(wrench.lease_id, authority.lease_id)
        self.assertGreater(wrench.sequence, authority.sequence)
