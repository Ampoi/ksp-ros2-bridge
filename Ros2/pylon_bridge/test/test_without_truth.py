import unittest
from unittest.mock import Mock
import rclpy
from pylon_bridge.udp_bridge import PyLoNBridge, parse_args


def heartbeat(generation=1,vessel='a'*32,**values):
    return dict(type='pylon_session',version=1,vesselId=vessel,universalTime=float(generation),
                runtimeInstance='instance',runtimeGeneration=generation,runtimeEpoch=f'epoch{generation}',
                runtimeVesselId=vessel,vesselName='test',available=True,**values)

class WithoutTruthTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(domain_id=174)
        self.addCleanup(rclpy.shutdown)
        self.node=PyLoNBridge(parse_args(['--host','127.0.0.1','--port','0','--disable-ground-truth']))
        self.addCleanup(self.node.destroy_node)

    def test_session_drives_lifecycle_without_any_sensor_or_truth(self):
        node=self.node;node.vessel_lifecycle_publisher=Mock()
        node.vehicle_state.publish_ground_truth(object())
        node.vehicle_state.publish_nearby_vessels(object())
        node.vehicle_state.publish_ground_truth_transform_at(0.,None)
        self.assertIsNone(node.latest_ground_truth)
        self.assertIsNone(node.ground_truth_pose_publisher)
        self.assertIsNone(node.nearby_vessels_publisher)
        node.flight.observe_session(heartbeat())
        lifecycle=node.vessel_lifecycle_publisher.publish.call_args.args[0]
        self.assertEqual((lifecycle.vessel_id,lifecycle.generation,lifecycle.world_frame),('a'*32,node.session.generation,''))
        node.flight.observe_session(heartbeat(2,'b'*32))
        self.assertEqual(node.active_vessel_id,'b'*32)
        self.assertEqual(node.vessel_generation,node.session.generation)

    def test_epoch_change_clears_all_buffered_state_and_rejects_old_data(self):
        node=self.node;first=heartbeat();node.flight.observe_session(first)
        node.motor_states['old']=object()
        node.latest_ground_truth=object();node.static_sensor_transforms['old']=object()
        node.flight.observe_session(heartbeat(2))
        self.assertEqual(node.vessel_generation,node.session.generation)
        self.assertIsNone(node.latest_ground_truth)
        self.assertFalse(node.motor_states)
        self.assertFalse(node.static_sensor_transforms)
        self.assertFalse(node.session.accepts(dict(first,type='pylon_imu')))
        node.flight.observe_session(first)
        self.assertEqual(node.vessel_generation,node.session.generation)

    def test_truth_mode_uses_the_same_identity_path(self):
        self.node.ground_truth_enabled=True
        self.node.flight.observe_session(heartbeat())
        self.assertEqual(self.node.active_vessel_id,'a'*32)
        self.assertIsNone(self.node.latest_ground_truth)

    def test_reordered_imu_does_not_publish_a_negative_time_interval(self):
        n=self.node;n.flight.observe_session(heartbeat());n.imu_publisher=Mock()
        data=dict(type='pylon_imu',version=1,vesselId='a'*32,universalTime=10.,
                  angularVelocity=[0.,0.,1.],linearAcceleration=[0.,0.,0.])
        n.sensors.publish_imu(data)
        n.sensors.publish_imu(dict(data,universalTime=9.))
        n.sensors.publish_imu(dict(data,universalTime=10.))
        self.assertEqual(n.imu_publisher.publish.call_count,1)
        n.flight.observe_session(heartbeat(2))
        n.sensors.publish_imu(dict(data,universalTime=1.))
        self.assertEqual(n.imu_publisher.publish.call_count,2)

    def test_model_clear_notification_carries_the_new_session_consistently(self):
        n=self.node;n.flight.observe_session(heartbeat())
        n.active_model=object();n.vessel_lifecycle_publisher=Mock()
        n.flight.observe_session(heartbeat(2,'b'*32))
        self.assertGreater(n.vessel_lifecycle_publisher.publish.call_count,1)
        for call in n.vessel_lifecycle_publisher.publish.call_args_list:
            message=call.args[0]
            self.assertEqual(message.vessel_id,'b'*32)
            self.assertEqual(message.runtime_epoch,'epoch2')
            self.assertEqual(message.generation,n.session.generation)
