import math
import unittest

import numpy as np

from pylon_demo_position_estimator.core import (
    attitude_drift_angle,
    default_ground_truth_pose_topic,
    default_lidar_topic,
    filter_range,
    interpolate_samples,
    iterative_closest_point,
    matrix_to_quaternion,
    position_drift,
    quaternion_angle,
    quaternion_conjugate,
    quaternion_multiply,
    rigid_transform_3d,
    rotation_angle,
    sanitize_ros_component,
    transform_points,
    twist_from_delta,
    voxel_downsample,
)


def _grid_cloud() -> np.ndarray:
    rng = np.random.default_rng(42)
    return rng.uniform(-2.0, 2.0, size=(600, 3))


class CoreTests(unittest.TestCase):
    def test_sanitize_matches_bridge_rules(self):
        self.assertEqual(sanitize_ros_component("Front-Lidar!", "lidar"), "front_lidar")
        self.assertEqual(
            default_lidar_topic("/ksp_vessel", "Front-Lidar!"),
            "/ksp_vessel/lidar_3d/front_lidar/points",
        )

    def test_filter_range_and_voxel(self):
        points = np.array(
            [
                [0.5, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [200.0, 0.0, 0.0],
                [math.nan, 0.0, 0.0],
            ]
        )
        filtered = filter_range(points, 0.8, 150.0)
        self.assertEqual(filtered.shape, (1, 3))
        dense = np.array([[0.05 * i, 0.0, 0.0] for i in range(20)])
        downsampled = voxel_downsample(dense, voxel_size=0.5, max_points=4)
        self.assertLessEqual(downsampled.shape[0], 4)
        self.assertGreater(downsampled.shape[0], 0)

    def test_rigid_fit_recovers_known_transform(self):
        source = _grid_cloud()
        angle = math.radians(10.0)
        rotation = np.array(
            [
                [math.cos(angle), -math.sin(angle), 0.0],
                [math.sin(angle), math.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        expected = np.eye(4)
        expected[:3, :3] = rotation
        expected[:3, 3] = np.array([0.4, -0.2, 0.1])
        target = transform_points(source, expected)
        recovered = rigid_transform_3d(source, target)
        np.testing.assert_allclose(recovered, expected, atol=1e-9)
        self.assertAlmostEqual(rotation_angle(expected), angle, places=9)

    def test_icp_converges_on_shifted_cloud(self):
        previous = _grid_cloud()
        shift = np.eye(4)
        shift[:3, 3] = np.array([0.3, 0.1, -0.05])
        current = transform_points(previous, np.linalg.inv(shift))
        delta, rmse, matches = iterative_closest_point(
            current,
            previous,
            np.eye(4),
            max_iterations=30,
            max_correspondence_distance=1.0,
            trim_fraction=1.0,
            minimum_correspondences=30,
        )
        np.testing.assert_allclose(delta[:3, 3], shift[:3, 3], atol=0.05)
        self.assertLess(rmse, 0.1)
        self.assertGreaterEqual(matches, 30)

    def test_icp_rejects_too_few_points(self):
        with self.assertRaises(ValueError):
            iterative_closest_point(
                np.zeros((2, 3)), np.zeros((40, 3)), np.eye(4)
            )

    def test_twist_sign_convention_moves_forward(self):
        # Sensor moved +0.5 m along X between scans: current points appear
        # shifted by -0.5 m in the current frame, so the ICP delta that maps
        # current -> previous carries t = (-0.5, 0, 0) with identity rotation.
        delta = np.eye(4)
        delta[0, 3] = -0.5
        twist = twist_from_delta(delta, 0.5)
        self.assertAlmostEqual(twist[0], 1.0, places=9)
        self.assertAlmostEqual(twist[3], 0.0, places=9)

    def test_matrix_to_quaternion_round_trip(self):
        angle = math.radians(30.0)
        rotation = np.array(
            [
                [math.cos(angle), -math.sin(angle), 0.0],
                [math.sin(angle), math.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        quat = matrix_to_quaternion(rotation)
        self.assertAlmostEqual(
            quat[3], math.cos(angle / 2.0), places=9
        )
        self.assertAlmostEqual(
            quat[2], math.sin(angle / 2.0), places=9
        )

    def test_quaternion_helpers_compose_and_invert(self):
        angle = math.radians(45.0)
        yaw = (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0))
        identity = (0.0, 0.0, 0.0, 1.0)
        composed = quaternion_multiply(quaternion_conjugate(yaw), yaw)
        for actual, expected in zip(composed, identity):
            self.assertAlmostEqual(actual, expected, places=9)
        self.assertAlmostEqual(quaternion_angle(yaw), angle, places=9)
        self.assertAlmostEqual(quaternion_angle(identity), 0.0, places=9)

    def test_ground_truth_topic_default(self):
        self.assertEqual(
            default_ground_truth_pose_topic("/ksp_vessel"),
            "/ksp_vessel/ground_truth/pose",
        )

    def test_interpolate_samples_midpoint_and_clamp(self):
        stamps = np.array([1.0, 2.0, 3.0])
        positions = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [4.0, 0.0, 0.0]])
        quats = np.array([[0.0, 0.0, 0.0, 1.0]] * 3)
        middle, _ = interpolate_samples(stamps, positions, quats, 1.5)
        np.testing.assert_allclose(middle, [1.0, 0.0, 0.0], atol=1e-9)
        before, _ = interpolate_samples(stamps, positions, quats, 0.0)
        np.testing.assert_allclose(before, [0.0, 0.0, 0.0], atol=1e-9)
        after, _ = interpolate_samples(stamps, positions, quats, 9.0)
        np.testing.assert_allclose(after, [4.0, 0.0, 0.0], atol=1e-9)
        with self.assertRaises(ValueError):
            interpolate_samples(np.array([]), positions[:0], quats[:0], 1.0)

    def test_drift_errors_are_zero_when_trajectories_match(self):
        est = np.array([5.0, 1.0, -2.0])
        est0 = np.array([1.0, 1.0, 1.0])
        gt = np.array([105.0, 1.0, -2.0])
        gt0 = np.array([101.0, 1.0, 1.0])
        error, norm = position_drift(est, est0, gt, gt0)
        np.testing.assert_allclose(error, [0.0, 0.0, 0.0], atol=1e-9)
        self.assertAlmostEqual(norm, 0.0, places=9)
        angle = math.radians(20.0)
        yaw = (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0))
        identity = (0.0, 0.0, 0.0, 1.0)
        # Same relative rotation from different absolute origins: no drift.
        self.assertAlmostEqual(
            attitude_drift_angle(yaw, identity, yaw, identity), 0.0, places=9
        )
        self.assertAlmostEqual(
            attitude_drift_angle(yaw, identity, identity, identity), angle, places=9
        )


if __name__ == "__main__":
    unittest.main()
