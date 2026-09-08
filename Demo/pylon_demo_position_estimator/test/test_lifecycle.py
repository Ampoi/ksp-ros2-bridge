import unittest
import numpy as np
import rclpy
from pylon_interfaces.msg import VesselLifecycle
from pylon_demo_position_estimator.node import PositionEstimatorNode

class LifecycleTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(domain_id=179)
        self.addCleanup(rclpy.shutdown)
        self.node=PositionEstimatorNode()
        self.addCleanup(self.node.destroy_node)

    def test_cloud_is_ignored_until_an_active_session_exists(self):
        self.node.receive_cloud(None)
        self.assertIsNone(self.node.previous_points)

    def test_same_vessel_new_generation_clears_registration_and_evaluation(self):
        n=self.node;n.receive_lifecycle(VesselLifecycle(vessel_id='same',generation=1,state=1))
        n.previous_points=np.ones((4,3));n.pose[0,3]=9;n.sequence=77
        n.receive_lifecycle(VesselLifecycle(vessel_id='same',generation=1,state=1))
        self.assertEqual(n.sequence,77)
        n.receive_lifecycle(VesselLifecycle(vessel_id='same',generation=2,state=1))
        self.assertIsNone(n.previous_points)
        self.assertEqual(n.sequence,0)
        np.testing.assert_array_equal(n.pose,np.eye(4))
