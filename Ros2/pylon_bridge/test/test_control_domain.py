import unittest

from pylon_bridge.domain.control import (
    authority_state_from_packet,
    wrench_feedback_from_packet,
)


class ControlDomainTests(unittest.TestCase):
    def test_parses_owned_authority_snapshot(self):
        state = authority_state_from_packet(
            {
                "type": "pylon_control_authority_state",
                "version": 1,
                "state": 1,
                "vesselId": "vessel",
                "vessel": "Inspector",
                "controllerId": "controller",
                "leaseId": "lease",
                "priority": 100,
                "leaseRemainingSeconds": 1.25,
                "sasSuppressed": True,
                "emergencyStop": False,
                "lastSequence": 4,
                "reason": "lease_acquired",
            }
        )
        self.assertEqual(state.vessel_id, "vessel")
        self.assertEqual(state.lease_remaining, 1.25)
        self.assertTrue(state.sas_suppressed)

    def test_parses_requested_allocated_achieved_feedback(self):
        zero = {"force": [0.0, 0.0, 0.0], "torque": [0.0, 0.0, 0.0]}
        requested = {"force": [10.0, 0.0, 0.0], "torque": [0.0, 2.0, 0.0]}
        feedback = wrench_feedback_from_packet(
            {
                "type": "pylon_wrench_status",
                "version": 1,
                "vesselId": "vessel",
                "controllerId": "controller",
                "leaseId": "lease",
                "sequence": 5,
                "accepted": True,
                "reason": "accepted",
                "requested": requested,
                "allocated": requested,
                "achieved": zero,
                "allocationResidual": zero,
                "trackingResidual": requested,
                "saturationRatio": 0.0,
                "trackingErrorRatio": 1.0,
                "saturated": False,
                "achievedQuality": "previous_physics_tick",
            }
        )
        self.assertEqual(feedback.requested.force, (10.0, 0.0, 0.0))
        self.assertEqual(feedback.tracking_error_ratio, 1.0)

    def test_rejects_nonfinite_feedback(self):
        zero = {"force": [0.0, 0.0, 0.0], "torque": [0.0, 0.0, 0.0]}
        packet = {
            "type": "pylon_wrench_status",
            "version": 1,
            "vesselId": "vessel",
            "requested": zero,
            "allocated": zero,
            "achieved": zero,
            "allocationResidual": zero,
            "trackingResidual": zero,
            "saturationRatio": float("nan"),
            "trackingErrorRatio": 0.0,
        }
        with self.assertRaisesRegex(ValueError, "saturationRatio"):
            wrench_feedback_from_packet(packet)


if __name__ == "__main__":
    unittest.main()
