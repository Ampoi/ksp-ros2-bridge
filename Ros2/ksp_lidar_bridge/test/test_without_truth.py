import unittest
from unittest.mock import Mock

try:
    import rclpy
    from ksp_lidar_bridge.udp_bridge import KerbalLidarUdpBridge, parse_args
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'source ROS Jazzy to test bridge without truth')
class WithoutTruthTests(unittest.TestCase):
    def test_imu_identity_drives_lifecycle_while_truth_packets_are_discarded(self):
        rclpy.init(domain_id=174)
        node = KerbalLidarUdpBridge(parse_args(['--host','127.0.0.1','--port','0','--disable-ground-truth']))
        try:
            node.vessel_lifecycle_publisher = Mock()
            node.publish_ground_truth(object())
            node.publish_nearby_vessels(object())
            node.publish_ground_truth_transform_at(0., None)
            self.assertIsNone(node.latest_ground_truth)
            self.assertIsNone(node.ground_truth_pose_publisher)
            self.assertIsNone(node.nearby_vessels_publisher)
            self.assertIsNone(node.ground_truth_frame_rate_publisher)
            packet = dict(type='ksp_imu',version=1,vesselId='a'*32,universalTime=1.,
                          angularVelocity=[0,0,0],linearAcceleration=[0,0,0])
            node.publish_imu(packet)
            lifecycle = node.vessel_lifecycle_publisher.publish.call_args.args[0]
            self.assertEqual(lifecycle.vessel_id,'a'*32)
            self.assertEqual(lifecycle.origin_sequence,1)
            self.assertEqual(lifecycle.world_frame,'')
            packet['vesselId'] = 'b'*32
            node.publish_imu(packet)
            self.assertEqual(node.vessel_lifecycle_publisher.publish.call_args.args[0].origin_sequence,2)
        finally:
            node.sock.close()
            node.destroy_node()
            rclpy.shutdown()

    def test_teleport_epoch_resets_lifecycle_without_truth_or_vessel_change(self):
        rclpy.init(domain_id=174)
        node = KerbalLidarUdpBridge(parse_args(['--host','127.0.0.1','--port','0','--disable-ground-truth']))
        try:
            packet=dict(type='ksp_imu',version=1,vesselId='a'*32,universalTime=1.,
                        runtimeEpoch='before',angularVelocity=[0,0,0],linearAcceleration=[0,0,1.63])
            node.publish_imu(packet);first=node.vessel_generation
            packet.update(runtimeEpoch='after',universalTime=2.)
            node.publish_imu(packet)
            self.assertGreater(node.vessel_generation,first)
            self.assertIsNone(node.latest_ground_truth)
        finally:
            node.sock.close();node.destroy_node();rclpy.shutdown()
