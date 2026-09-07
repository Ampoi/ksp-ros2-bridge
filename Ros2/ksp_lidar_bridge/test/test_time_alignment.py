import math
import unittest

from ksp_lidar_bridge.domain.time_alignment import SimulationClock, extrapolate_pose


class ContinuousImuClockTests(unittest.TestCase):
    def test_delayed_packets_and_slow_physics_do_not_change_imu_intervals(self):
        clock = SimulationClock(continuous=True)
        first = clock.map_nanoseconds(10., 100_000_000_000)
        clock.map_nanoseconds(10.01, 102_000_000_000)  # delayed camera
        second = clock.map_nanoseconds(10.04, 102_100_000_000)
        self.assertAlmostEqual((second-first)*1e-9, .04, places=6)
        clock.reset()
        self.assertEqual(clock.map_nanoseconds(1., 110_000_000_000), 110_000_000_000)


class TimeAlignmentTests(unittest.TestCase):
    def test_maps_ksp_time_with_stable_receipt_offset(self):
        clock = SimulationClock()
        self.assertEqual(clock.map_nanoseconds(100.0, 1_000_000_000), 1_000_000_000)
        self.assertEqual(clock.map_nanoseconds(100.1, 1_100_000_000), 1_100_000_000)

    def test_extrapolates_pose_to_sensor_timestamp(self):
        position, orientation = extrapolate_pose(
            (1.0, 2.0, 3.0),
            (0.0, 0.0, 0.0, 1.0),
            (2.0, 0.0, 0.0),
            (0.0, 0.0, math.pi),
            0.1,
        )
        self.assertAlmostEqual(position[0], 1.2)
        self.assertAlmostEqual(orientation[2], math.sin(math.pi * 0.05))
        self.assertAlmostEqual(orientation[3], math.cos(math.pi * 0.05))

    def test_extrapolation_is_bounded(self):
        position, _ = extrapolate_pose(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            10.0,
        )
        self.assertEqual(position, (0.1, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
