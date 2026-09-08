from pylon_bridge.services.model import ModelService
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

try:
    from builtin_interfaces.msg import Time
    from pylon_bridge.udp_bridge import PyLoNBridge
except ImportError:
    PyLoNBridge = None


@unittest.skipIf(PyLoNBridge is None, 'source ROS Jazzy to test camera TF')
class CameraTransformTests(unittest.TestCase):
    def test_moving_camera_preserves_each_image_timestamp(self):
        node = Mock()
        node.args = SimpleNamespace(frame_prefix='test')
        node.model = ModelService(node)
        node.model.model_frame_for_packet = Mock()
        node.model.model_frame_for_packet.return_value = 'camera_part'
        node.static_sensor_transforms = {}
        for seconds, rotation in [(1, [0., 0., 0., 1.]),
                                  (2, [0., 0., 1., 0.]),
                                  (3, [0., 0., 1., 0.])]:
            packet = dict(coordinateFrame='ros_sensor', framePosition=[0., 0., 0.1],
                          frameRotation=rotation)
            node.model.sensor_frame_for_packet( packet, 'front_camera', Time(sec=seconds), 'camera_optical_frame', dynamic=True)
        sent = [call.args[0] for call in node.transform_broadcaster.sendTransform.call_args_list]
        self.assertEqual([m.header.stamp.sec for m in sent], [1, 2, 3])
        self.assertEqual([m.transform.rotation.z for m in sent], [0., 1., 1.])
        self.assertEqual({m.header.frame_id for m in sent}, {'camera_part'})
        node.static_transform_broadcaster.sendTransform.assert_not_called()

    def test_fixed_lidar_still_uses_static_snapshot(self):
        node = Mock()
        node.args = SimpleNamespace(frame_prefix='test')
        node.model = ModelService(node)
        node.model.model_frame_for_packet = Mock()
        node.model.model_frame_for_packet.return_value = 'lidar_part'
        node.static_sensor_transforms = {}
        packet = dict(coordinateFrame='ros_sensor', framePosition=[0., 0., 0.1],
                      frameRotation=[0., 0., 0., 1.])
        for second in (1, 2):
            node.model.sensor_frame_for_packet( packet, 'lidar', Time(sec=second))
        node.static_transform_broadcaster.sendTransform.assert_called_once()
        node.transform_broadcaster.sendTransform.assert_not_called()
