import math
import unittest

from ksp_vehicle_control.domain.control_law import (
    body_detumble_torque,
    body_wrench_for_setpoint,
    rate_guard_body_torque,
)


class ControlLawTests(unittest.TestCase):
    def test_world_force_is_rotated_into_body(self):
        half = math.sqrt(0.5)
        force, torque = body_wrench_for_setpoint(
            (0, 0, 0), (0, 0, half, half), (0, 0, 0), (0, 0, 0),
            (1, 0, 0), (0, 0, half, half), (0, 0, 0), (0, 0, 0),
            1, 1, 1, 1, 10, 10, True,
        )
        self.assertAlmostEqual(force[0], 0.0, places=6)
        self.assertAlmostEqual(force[1], -1.0, places=6)
        self.assertEqual(torque, (0.0, 0.0, 0.0))

    def test_detumble_opposes_world_rate_in_body(self):
        torque = body_detumble_torque((0, 0, 2), 2, 1)
        self.assertEqual(torque, (0.0, 0.0, -1.0))

    def test_rate_guard_removes_accelerating_torque(self):
        torque = rate_guard_body_torque(
            (0, 0, 1), (0, 0, 2), 1, 2, 3
        )
        self.assertLess(torque[2], 0.0)

    def test_large_attitude_step_is_converted_to_bounded_rate(self):
        half = math.sqrt(0.5)
        _, torque_at_rest = body_wrench_for_setpoint(
            (0, 0, 0), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0),
            (0, 0, 0), (0, 0, half, half), (0, 0, 0), (0, 0, 0),
            1, 1, 80, 20, 10, 10, False, 0.05,
        )
        _, torque_at_limit = body_wrench_for_setpoint(
            (0, 0, 0), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0.05),
            (0, 0, 0), (0, 0, half, half), (0, 0, 0), (0, 0, 0),
            1, 1, 80, 20, 10, 10, False, 0.05,
        )
        self.assertAlmostEqual(torque_at_rest[2], 1.0)
        self.assertAlmostEqual(torque_at_limit[2], 0.0)


if __name__ == "__main__":
    unittest.main()
