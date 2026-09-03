import math
import unittest

from debris_orbit.core import (
    body_orientation_for_sensor_look_at,
    detumble_required,
    euclidean_clusters,
    integrate_world_orientation,
    linear_ramp_fraction,
    look_at_quaternion,
    png_bytes,
    rotate_vector,
    safe_filename_component,
    search_direction,
    unwrap_angle,
    vessel_topics,
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

    def test_sensor_mount_is_compensated_when_pointing_at_target(self):
        sensor_in_body = look_at_quaternion((0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        body_world = body_orientation_for_sensor_look_at(
            (0.0, 0.0, 1.0), (0.0, 1.0, 0.0), sensor_in_body
        )
        sensor_forward_body = rotate_vector(sensor_in_body, (1.0, 0.0, 0.0))
        sensor_forward_world = rotate_vector(body_world, sensor_forward_body)
        for actual, expected in zip(sensor_forward_world, (0.0, 0.0, 1.0)):
            self.assertAlmostEqual(actual, expected, places=7)

    def test_search_is_bounded_periodic_and_not_continuously_rotating(self):
        center = (1.0, 0.0, 0.0)
        start = search_direction(center, (0.0, 0.0, 1.0), 0.0, 8.0, 0.2, 0.1)
        after_period = search_direction(center, (0.0, 0.0, 1.0), 8.0, 8.0, 0.2, 0.1)
        half_period = search_direction(center, (0.0, 0.0, 1.0), 4.0, 8.0, 0.2, 0.1)
        positive = search_direction(center, (0.0, 0.0, 1.0), 2.0, 8.0, 0.2, 0.1)
        negative = search_direction(center, (0.0, 0.0, 1.0), 6.0, 8.0, 0.2, 0.1)
        for actual, expected in zip(start, after_period):
            self.assertAlmostEqual(actual, expected, places=7)
        self.assertAlmostEqual(start[1], 0.0, places=7)
        self.assertAlmostEqual(half_period[1], 0.0, places=7)
        self.assertGreater(positive[1], 0.0)
        self.assertLess(negative[1], 0.0)

    def test_unwrap_crosses_positive_pi_without_reversing(self):
        previous = math.radians(179.0)
        actual = unwrap_angle(previous, math.radians(-179.0))
        self.assertAlmostEqual(actual, math.radians(181.0))

    def test_integrates_world_frame_angular_velocity(self):
        orientation = integrate_world_orientation(
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, math.pi / 2.0),
            1.0,
        )
        result = rotate_vector(orientation, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(result[0], 0.0, places=7)
        self.assertAlmostEqual(result[1], 1.0, places=7)
        self.assertAlmostEqual(result[2], 0.0, places=7)

    def test_command_ramp_starts_at_zero_and_is_bounded(self):
        self.assertEqual(linear_ramp_fraction(0.0, 2.0), 0.0)
        self.assertEqual(linear_ramp_fraction(0.5, 2.0), 0.25)
        self.assertEqual(linear_ramp_fraction(3.0, 2.0), 1.0)

    def test_detumble_hysteresis_prevents_mode_chatter(self):
        self.assertTrue(detumble_required(False, 0.3, 0.2, 0.1))
        self.assertTrue(detumble_required(True, 0.15, 0.2, 0.1))
        self.assertFalse(detumble_required(True, 0.05, 0.2, 0.1))

    def test_png_encoder_outputs_png_signature(self):
        encoded = png_bytes(1, 1, "rgb8", 3, bytes((255, 0, 0)))
        self.assertTrue(encoded.startswith(b"\x89PNG\r\n\x1a\n"))
        self.assertIn(b"IHDR", encoded)
        self.assertIn(b"IEND", encoded)

    def test_filename_components_cannot_create_subdirectories(self):
        self.assertEqual(safe_filename_component("../debris / 1"), "debris_1")

    def test_resolves_current_ksp_vessel_topic_contract(self):
        topics = vessel_topics(
            "/ksp_vessel/", "Front LiDAR", "Orbit Camera", "Inspector 1"
        )
        self.assertEqual(
            topics.lidar_points,
            "/ksp_vessel/lidar_3d/front_lidar/points",
        )
        self.assertEqual(
            topics.camera_image,
            "/ksp_vessel/camera/orbit_camera/image_raw",
        )
        self.assertEqual(
            topics.ground_truth_pose,
            "/ksp_vessel/ground_truth/pose",
        )
        self.assertEqual(topics.body_wrench, "/ksp_vessel/body_wrench")
        self.assertEqual(
            topics.control_setpoint,
            "/ksp_vessel/demos/debris_orbit/inspector_1/setpoint",
        )
        self.assertEqual(
            topics.demo_status,
            "/ksp_vessel/demos/debris_orbit/inspector_1/status",
        )


if __name__ == "__main__":
    unittest.main()
