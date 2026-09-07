import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from debris_orbit.inertial import ImuIntegrator
from debris_orbit.perception import RelativeTracker
from debris_orbit.core import search_direction


class InertialTests(unittest.TestCase):
    def test_initial_search_reaches_behind_the_sensor(self):
        direction = search_direction((1,0,0),(0,0,1),120.,480.,np.pi,np.deg2rad(20))
        np.testing.assert_allclose(direction,[-1,0,0],atol=1e-8)

    def test_gyro_integrates_body_rate_without_absolute_orientation(self):
        imu = ImuIntegrator()
        for t in np.linspace(0, 2, 201):
            imu.push(t, [0, 0, np.pi/4], [0, 0, 0])
        np.testing.assert_allclose(imu.at(1).rotation.apply([1,0,0]), [2**-.5,2**-.5,0], atol=1e-8)
        np.testing.assert_allclose(imu.at(2).rotation.apply([1,0,0]), [0,1,0], atol=1e-8)

    def test_specific_force_predicts_target_minus_chaser_with_opposite_sign(self):
        imu = ImuIntegrator()
        for t in np.linspace(0, 2, 201):
            imu.push(t, [0,0,0], [2,0,0])
        dp, dv = imu.relative_motion(.25, 1.25)
        np.testing.assert_allclose(dp, [-1,0,0], atol=1e-8)
        np.testing.assert_allclose(dv, [-2,0,0], atol=1e-8)

    def test_out_of_order_is_ignored_and_missing_gyro_interval_is_not_integrated(self):
        imu = ImuIntegrator()
        self.assertTrue(imu.push(1, [0,0,1], [0,0,0]))
        self.assertFalse(imu.push(.9, [9,9,9], [0,0,0]))
        with self.assertRaises(ValueError):
            imu.push(2, [0,0,1], [0,0,0])
        self.assertEqual(len(imu.samples), 1)

    def test_thrust_acceleration_fusion_tracks_velocity_without_differentiating_noisy_positions(self):
        tracker = RelativeTracker(measurement_sigma=.05)
        tracker.update([[15,0,0]], 0.)
        for i in range(1, 21):
            t = i*.1
            out = tracker.update([[15-.5*t*t,0,0]], t,
                                 input_delta=(np.array([-.005,0,0]), np.array([-.1,0,0])))
        np.testing.assert_allclose(out[1], [13,0,0], atol=1e-8)
        np.testing.assert_allclose(out[2], [-2,0,0], atol=1e-8)
