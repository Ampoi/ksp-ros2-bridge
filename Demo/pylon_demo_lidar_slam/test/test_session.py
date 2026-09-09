import unittest

from pylon_demo_lidar_slam.session import SessionGuard
from pylon_vehicle_control.application.lease import LeaseCoordinator


class SessionTests(unittest.TestCase):
    def test_waits_for_active_vessel(self):
        guard = SessionGuard()
        guard.observe('', False, 0)
        self.assertFalse(guard.ready)
        self.assertFalse(guard.fault)
        guard.observe('vessel', True, 1)
        self.assertTrue(guard.ready)
        guard.observe('vessel', True, 1)
        self.assertTrue(guard.ready)

    def test_generation_change_latches_even_for_same_vessel(self):
        guard = SessionGuard()
        guard.observe('vessel', True, 1)
        guard.observe('vessel', True, 2)
        self.assertEqual(guard.fault, 'session_changed')
        guard.observe('vessel', True, 1)
        self.assertFalse(guard.ready)

    def test_new_vessel_requires_restart(self):
        guard = SessionGuard()
        guard.observe('first', True, 1)
        guard.observe('second', True, 1)
        self.assertFalse(guard.ready)

    def test_outage_does_not_resume_on_heartbeat(self):
        guard = SessionGuard()
        guard.observe('vessel', True, 1)
        guard.observe('vessel', False, 1)
        guard.observe('vessel', True, 1)
        self.assertEqual(guard.fault, 'session_unavailable')
        self.assertFalse(guard.ready)

    def test_control_fault_survives_lifecycle_updates(self):
        guard = SessionGuard()
        guard.observe('vessel', True, 1)
        guard.stop('odometry_timeout')
        guard.observe('vessel', True, 1)
        self.assertFalse(guard.ready)

    def test_current_lease_rejects_confirmation_from_previous_generation(self):
        lease = LeaseCoordinator('slam', 0.5)
        lease.observe_vessel('vessel', True, 1)
        first = lease.due_action(0.0)
        lease.observe_authority('vessel', 'slam', first.lease_id, True)
        self.assertTrue(lease.owned)
        lease.observe_vessel('vessel', True, 2)
        lease.observe_authority('vessel', 'slam', first.lease_id, True)
        self.assertFalse(lease.owned)
        self.assertNotEqual(lease.due_action(0.0).lease_id, first.lease_id)
