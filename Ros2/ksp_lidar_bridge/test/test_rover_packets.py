import unittest
from ksp_lidar_bridge.vehicle_packets import actuator_command,actuator_state_from_packet

class RoverPackets(unittest.TestCase):
    def test_brake_is_bounded_and_old_command_defaults_to_zero(self):
        for brake in (-.1,1.1,float('nan')):
            with self.assertRaises(ValueError):actuator_command('wheel','wheel',{'brake':brake},1,'v','c','l')
        self.assertEqual(actuator_command('wheel','wheel',{},1,'v','c','l')['brake'],0.)
        self.assertEqual(actuator_command('wheel','wheel',{'brake':1.},1,'v','c','l')['brake'],1.)
    def test_missing_geometry_cannot_be_mistaken_for_valid_radius(self):
        m=actuator_state_from_packet(dict(type='ksp_actuator_state',version=1,actuatorType='wheel',name='w'))
        self.assertEqual(m['radius'],0.);self.assertEqual(m['wheelCount'],0)
    def test_geometry_nan_rejected(self):
        p=dict(type='ksp_actuator_state',version=1,actuatorType='wheel',name='w',radius=float('nan'))
        with self.assertRaises(ValueError):actuator_state_from_packet(p)
