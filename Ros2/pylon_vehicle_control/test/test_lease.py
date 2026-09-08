import unittest

from pylon_vehicle_control.application.lease import LeaseCoordinator


class LeaseCoordinatorTests(unittest.TestCase):
    def test_waits_for_vessel_and_authoritative_ack(self):
        lease = LeaseCoordinator("controller", 0.5)
        self.assertIsNone(lease.due_action(0.0))
        lease.observe_vessel("vessel", True)
        request = lease.due_action(0.0)
        self.assertEqual(request.action, "acquire")
        lease.observe_authority("vessel", "controller", request.lease_id, True)
        self.assertTrue(lease.owned)
        self.assertIsNone(lease.due_action(0.4))
        self.assertEqual(lease.due_action(0.5).action, "renew")

    def test_vessel_change_invalidates_lease(self):
        lease = LeaseCoordinator("controller", 0.5)
        lease.observe_vessel("first", True)
        original = lease.lease_id
        lease.observe_authority("first", "controller", original, True)
        lease.observe_vessel("second", True)
        self.assertFalse(lease.owned)
        self.assertNotEqual(lease.lease_id, original)


if __name__ == "__main__":
    unittest.main()
