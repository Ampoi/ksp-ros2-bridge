import math
import unittest

from ksp_vehicle_control.domain.control_law import (
    body_detumble_torque,
    body_wrench_for_setpoint,
    rate_guard_body_torque,
)
from debris_orbit.core import look_at_quaternion


class ControlTests(unittest.TestCase):
    def test_detumble_torque_always_opposes_angular_velocity(self):
        torque = body_detumble_torque(
            (2.0, -3.0, 4.0), 10.0, 20.0
        )
        self.assertLess(2.0 * torque[0] - 3.0 * torque[1] + 4.0 * torque[2], 0.0)
        self.assertLessEqual(math.sqrt(sum(value * value for value in torque)), 20.0)

    def test_attitude_hold_never_requests_translation(self):
        desired = look_at_quaternion((0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        force, torque = body_wrench_for_setpoint(
            (100.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            (8.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            desired,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            120.0,
            350.0,
            800.0,
            300.0,
            5000.0,
            300.0,
            False,
        )
        self.assertEqual(force, (0.0, 0.0, 0.0))
        self.assertGreater(math.sqrt(sum(value * value for value in torque)), 0.0)

    def test_rate_guard_brakes_instead_of_accelerating_above_limit(self):
        torque = rate_guard_body_torque(
            (0.5, 0.2, 0.0),
            (0.1, 0.0, 0.0),
            rate_limit=0.05,
            braking_gain=4.0,
            maximum=0.5,
        )
        self.assertLess(torque[0], 0.0)
        self.assertLessEqual(math.sqrt(sum(value * value for value in torque)), 0.5)

    def test_rate_guard_preserves_command_below_limit(self):
        torque = (0.4, -0.1, 0.05)
        self.assertEqual(
            rate_guard_body_torque(
                torque,
                (0.01, 0.0, 0.0),
                rate_limit=0.05,
                braking_gain=4.0,
                maximum=0.5,
            ),
            torque,
        )


if __name__ == "__main__":
    unittest.main()
