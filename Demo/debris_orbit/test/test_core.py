import math
import unittest

from debris_orbit.core import (
    euclidean_clusters,
    look_at_quaternion,
    png_bytes,
    rotate_vector,
    safe_filename_component,
    unwrap_angle,
    voxel_downsample,
)


class CoreTests(unittest.TestCase):
    def test_clusters_separate_objects_and_sort_largest_first(self):
        points = [(0.1 * i, 0.0, 0.0) for i in range(6)]
        points += [(10.0 + 0.1 * i, 0.0, 0.0) for i in range(4)]
        clusters = euclidean_clusters(points, tolerance=0.25, minimum_points=3)
        self.assertEqual([len(cluster) for cluster in clusters], [6, 4])

    def test_voxel_downsample_rejects_non_finite_and_bounds_output(self):
        points = [
            (0.1, 0.1, 0.1),
            (0.2, 0.2, 0.2),
            (math.nan, 0.0, 0.0),
            (2.0, 0.0, 0.0),
        ]
        self.assertEqual(
            voxel_downsample(points, voxel_size=1.0, max_points=10),
            [(0.1, 0.1, 0.1), (2.0, 0.0, 0.0)],
        )

    def test_look_at_points_body_x_at_target(self):
        orientation = look_at_quaternion((0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        result = rotate_vector(orientation, (1.0, 0.0, 0.0))
        for actual, expected in zip(result, (0.0, 1.0, 0.0)):
            self.assertAlmostEqual(actual, expected, places=7)

    def test_unwrap_crosses_positive_pi_without_reversing(self):
        previous = math.radians(179.0)
        actual = unwrap_angle(previous, math.radians(-179.0))
        self.assertAlmostEqual(actual, math.radians(181.0))

    def test_png_encoder_outputs_png_signature(self):
        encoded = png_bytes(1, 1, "rgb8", 3, bytes((255, 0, 0)))
        self.assertTrue(encoded.startswith(b"\x89PNG\r\n\x1a\n"))
        self.assertIn(b"IHDR", encoded)
        self.assertIn(b"IEND", encoded)

    def test_filename_components_cannot_create_subdirectories(self):
        self.assertEqual(safe_filename_component("../debris / 1"), "debris_1")


if __name__ == "__main__":
    unittest.main()
