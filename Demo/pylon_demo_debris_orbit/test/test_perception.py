import unittest
import numpy as np
from pylon_demo_debris_orbit.perception import extract_clusters, RelativeTracker


class PerceptionTests(unittest.TestCase):
    def test_clusters_separate_objects_and_filter_invalid_and_out_of_range(self):
        rng = np.random.default_rng(17)
        near = rng.uniform(-.4, .4, (100, 3)) + [15, 0, 0]
        far = rng.uniform(-.4, .4, (50, 3)) + [30, 4, 0]
        points = np.r_[near, far, [[np.nan, 0, 0], [1e4, 0, 0], [0, 0, 0]]]
        clusters = extract_clusters(points)
        self.assertEqual(len(clusters), 2)
        np.testing.assert_allclose(clusters[0].center, [15, 0, 0], atol=.1)
        np.testing.assert_allclose(clusters[1].center, [30, 4, 0], atol=.1)

    def test_filter_tracks_relative_motion_and_rejects_wrong_object(self):
        tracker = RelativeTracker(measurement_sigma=.1)
        rng = np.random.default_rng(8)
        velocity = np.array([.2, -.1, .05])
        for i in range(120):
            point = [15, 2, -1] + velocity * i * .1 + rng.normal(0, .03, 3)
            result = tracker.update([point, [80, 0, 0]], i*.1)
        np.testing.assert_allclose(result[2], velocity, atol=.08)
        before = tracker.state.copy()
        self.assertIsNone(tracker.update([[80, 0, 0]], 12.))
        self.assertIsNone(tracker.update([[15, 0, 0]], 10.))
        np.testing.assert_array_equal(tracker.state, before)
        self.assertTrue(np.all(np.linalg.eigvalsh(tracker.covariance) >= 0))
        result = tracker.update([[20, 0, 0]], 14.)
        self.assertEqual(tracker.observations, 1)
        np.testing.assert_array_equal(result[2], [0, 0, 0])

    def test_scene_plane_and_off_axis_object_do_not_replace_debris(self):
        rng = np.random.default_rng(21)
        debris = rng.uniform(-.4, .4, (100, 3)) + [15, 0, 0]
        off_axis = rng.uniform(-.4, .4, (200, 3)) + [15, 30, 0]
        y, z = np.meshgrid(np.arange(-6., 7.), np.arange(-6., 7.))
        plane = np.c_[np.full(y.size, 50.), y.ravel(), z.ravel()]
        clusters = extract_clusters(np.r_[debris, off_axis, plane])
        self.assertEqual(len(clusters), 1)
        np.testing.assert_allclose(clusters[0].center, [15, 0, 0], atol=.1)

    def test_partial_sparse_scans_do_not_create_a_target(self):
        self.assertEqual(extract_clusters([[10., 0., 0.], [11., 0., 0.]]), [])
        tracker = RelativeTracker()
        self.assertIsNone(tracker.update([], 0.))
        self.assertIsNone(tracker.update([[15, 0, 0]], 0., initial_index=1))
