import unittest
from unittest.mock import Mock
try:
    from geometry_msgs.msg import TransformStamped
    from pylon_bridge.static_transforms import StaticTransformSnapshot
except ImportError:
    StaticTransformSnapshot = None


@unittest.skipIf(StaticTransformSnapshot is None, 'requires ROS2 messages')
class StaticTransformTests(unittest.TestCase):
    def test_relaunch_reparents_same_sensor_and_updates_late_joiner_snapshot(self):
        node = Mock()
        broadcaster = StaticTransformSnapshot(node)
        old = TransformStamped(child_frame_id='lidar')
        old.header.frame_id = 'old_vessel_part'
        broadcaster.sendTransform(old)
        new = TransformStamped(child_frame_id='lidar')
        new.header.frame_id = 'new_vessel_part'
        broadcaster.sendTransform(new)
        message = broadcaster.publisher.publish.call_args.args[0]
        self.assertEqual(len(message.transforms), 1)
        self.assertEqual(message.transforms[0].header.frame_id, 'new_vessel_part')
        broadcaster.clear()
        self.assertEqual(broadcaster.publisher.publish.call_args.args[0].transforms, [])
