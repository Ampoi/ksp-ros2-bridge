import unittest
from unittest.mock import Mock
import rclpy
from pylon_interfaces.msg import ControlSetpoint, ControlAuthorityState, VesselLifecycle
from pylon_vehicle_control.adapters.ros2.setpoint_controller import SetpointController

class InterruptionTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(domain_id=176)
        self.addCleanup(rclpy.shutdown)
        self.node=SetpointController()
        self.addCleanup(self.node.destroy_node)
        self.node.receive_lifecycle(VesselLifecycle(vessel_id='vessel',generation=1,state=1))

    def test_authority_loss_requires_idle_before_new_setpoint(self):
        n=self.node
        n.lease.owned=True
        n.receive_authority(ControlAuthorityState(vessel_id='vessel',state=0))
        self.assertTrue(n.control_interrupted)
        n.receive_setpoint(ControlSetpoint(mode=ControlSetpoint.MODE_ATTITUDE_HOLD))
        self.assertIsNone(n.setpoint)
        n._maintain_authority=Mock();n.control();n._maintain_authority.assert_not_called()
        n.receive_setpoint(ControlSetpoint(mode=ControlSetpoint.MODE_IDLE))
        n.receive_setpoint(ControlSetpoint(mode=ControlSetpoint.MODE_ATTITUDE_HOLD))
        self.assertFalse(n.control_interrupted)
        self.assertIsNotNone(n.setpoint)

    def test_same_vessel_new_epoch_discards_goal_and_lease(self):
        n=self.node;old=n.lease.lease_id
        n.receive_setpoint(ControlSetpoint(mode=ControlSetpoint.MODE_ATTITUDE_HOLD))
        n.receive_lifecycle(VesselLifecycle(vessel_id='vessel',generation=2,state=1))
        self.assertTrue(n.control_interrupted)
        self.assertIsNone(n.setpoint)
        self.assertNotEqual(n.lease.lease_id,old)
