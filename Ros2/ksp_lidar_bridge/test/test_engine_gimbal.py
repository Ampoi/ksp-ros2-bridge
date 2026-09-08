import unittest

from ksp_lidar_bridge.vehicle_packets import actuator_command, actuator_state_from_packet


class EngineGimbalTests(unittest.TestCase):
    def command(self, **values):
        return actuator_command('engine', 'engine_123_0', values, 3, 'vessel', 'controller', 'lease')

    def test_opt_in_and_axis_mapping(self):
        command = self.command(hasGimbalCommand=True, gimbalPitch=-1., gimbalYaw=.25, gimbalRoll=1.)
        self.assertTrue(command['hasGimbalCommand'])
        self.assertEqual([command[x] for x in ('gimbalPitch', 'gimbalYaw', 'gimbalRoll')], [-1., .25, 1.])
        self.assertFalse(self.command(targetThrust=20.)['hasGimbalCommand'])

    def test_rejects_invalid_axes_even_when_disabled(self):
        for axis in ('gimbalPitch', 'gimbalYaw', 'gimbalRoll'):
            for value in (-1.01, 1.01, float('nan'), float('inf'), 'bad'):
                with self.subTest(axis=axis, value=value), self.assertRaises(ValueError):
                    self.command(**{axis: value})

    def test_state_and_old_packet_defaults(self):
        packet = dict(type='ksp_actuator_state', version=1, actuatorType='engine', name='engine_123_0')
        state = actuator_state_from_packet(packet)
        self.assertEqual(state['gimbalPitch'], 0.)
        packet.update(gimbalAvailable=True, gimbalCommandActive=True, gimbalPitch=.1, gimbalYaw=-.2, gimbalRoll=.3)
        state = actuator_state_from_packet(packet)
        self.assertTrue(state['gimbalAvailable'])
        self.assertEqual(state['gimbalYaw'], -.2)
        packet['gimbalRoll'] = float('nan')
        with self.assertRaises(ValueError):
            actuator_state_from_packet(packet)

    def test_ros_message_serialization(self):
        from ksp_ros2_interfaces.msg import EngineCommand, EngineState
        from rclpy.serialization import deserialize_message, serialize_message
        for cls in (EngineCommand, EngineState):
            message = cls(gimbal_pitch=.1, gimbal_yaw=-.2, gimbal_roll=.3)
            restored = deserialize_message(serialize_message(message), cls)
            self.assertEqual(restored.gimbal_pitch, .1)
            self.assertEqual(restored.gimbal_yaw, -.2)
            self.assertEqual(restored.gimbal_roll, .3)

    def test_bridge_command_and_state_mapping(self):
        from types import SimpleNamespace
        from builtin_interfaces.msg import Time
        from ksp_ros2_interfaces.msg import EngineCommand
        from ksp_lidar_bridge.udp_bridge import KerbalLidarUdpBridge
        sent, states = [], []
        fake = SimpleNamespace(
            actuator_kinds={}, command_sequence=0, actuator_last_seen={},
            args=SimpleNamespace(vehicle_command_timeout_sec=.5),
            send_vehicle_packet=lambda command, label: sent.append(command),
            ensure_actuator=lambda name, kind: SimpleNamespace(publish=states.append),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
            get_logger=lambda: SimpleNamespace(warning=lambda text: None),
        )
        command = EngineCommand(vessel_id='vessel', controller_id='controller', lease_id='lease',
                                sequence=3, id='engine_123_0', enabled=True, target_thrust=20.,
                                has_gimbal_command=True, gimbal_pitch=.1, gimbal_yaw=-.2, gimbal_roll=.3)
        KerbalLidarUdpBridge.send_typed_actuator_command(fake, 'engine', command)
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0]['gimbalYaw'], -.2)
        self.assertTrue(sent[0]['hasGimbalCommand'])
        KerbalLidarUdpBridge.publish_actuator_state(fake, dict(
            type='ksp_actuator_state', version=1, actuatorType='engine', name='engine_123_0',
            gimbalAvailable=True, gimbalCommandActive=True, gimbalPitch=.1, gimbalYaw=-.2, gimbalRoll=.3))
        self.assertEqual(len(states), 1)
        self.assertTrue(states[0].gimbal_available)
        self.assertTrue(states[0].gimbal_command_active)
        self.assertEqual(states[0].gimbal_yaw, -.2)
        command.gimbal_pitch = 1.1
        KerbalLidarUdpBridge.send_typed_actuator_command(fake, 'engine', command)
        self.assertEqual(len(sent), 1)
