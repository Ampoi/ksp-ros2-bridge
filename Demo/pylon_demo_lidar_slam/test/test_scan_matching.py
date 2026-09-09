import math
import unittest

import numpy as np

from pylon_demo_lidar_slam.scan_matching import (
    iterative_closest_point,
    laser_points,
    transform_points,
    yaw_from_transform,
)


class LaserPointTests(unittest.TestCase):
    def test_filters_invalid_ranges(self):
        points = laser_points(
            [1.0, math.inf, 3.0, -1.0],
            0.0,
            math.pi / 2.0,
            0.1,
            2.0,
            100,
        )
        np.testing.assert_allclose(points, [[1.0, 0.0]], atol=1e-8)

    def test_limits_point_count(self):
        points = laser_points([1.0] * 100, 0.0, 0.01, 0.1, 2.0, 20)
        self.assertEqual(points.shape, (20, 2))


class IcpTests(unittest.TestCase):
    def test_recovers_current_to_previous_transform(self):
        generator = np.random.default_rng(42)
        previous = generator.uniform((-6.0, -4.0), (7.0, 5.0), size=(240, 2))
        expected = np.array(
            [
                [math.cos(0.04), -math.sin(0.04), 0.15],
                [math.sin(0.04), math.cos(0.04), -0.08],
                [0.0, 0.0, 1.0],
            ]
        )
        current = transform_points(previous, np.linalg.inv(expected))
        actual, error, matches = iterative_closest_point(
            current,
            previous,
            np.eye(3),
            max_correspondence_distance=1.0,
            minimum_correspondences=30,
        )
        np.testing.assert_allclose(actual, expected, atol=0.025)
        self.assertLess(error, 0.05)
        self.assertGreater(matches, 100)
        self.assertAlmostEqual(yaw_from_transform(actual), 0.04, delta=0.02)

    def test_rejects_insufficient_scan(self):
        with self.assertRaisesRegex(ValueError, "too few"):
            iterative_closest_point(
                np.zeros((3, 2)), np.zeros((3, 2)), np.eye(3)
            )


if __name__ == "__main__":
    unittest.main()
