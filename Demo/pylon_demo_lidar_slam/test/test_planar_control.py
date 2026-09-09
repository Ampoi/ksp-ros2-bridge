import math
import unittest

from pylon_demo_lidar_slam.math_utils import planar_wrench, rotate_planar


class PlanarControlTests(unittest.TestCase):
    def test_computes_and_clamps_wrench(self):
        force, torque = planar_wrench(
            (2.0, -1.0, 3.0),
            (0.5, 0.0, 0.0),
            linear_gain=100.0,
            angular_gain=50.0,
            max_force=120.0,
            max_torque=80.0,
        )
        self.assertEqual(force, (120.0, -100.0, 0.0))
        self.assertEqual(torque, (0.0, 0.0, 80.0))

    def test_rotates_nav_force_into_body_frame(self):
        force = rotate_planar((1.0, 0.0, 0.0), math.pi / 2.0)
        self.assertAlmostEqual(force[0], 0.0, places=7)
        self.assertAlmostEqual(force[1], 1.0, places=7)


if __name__ == "__main__":
    unittest.main()
