from pylon_bridge.services.sensors import SensorsService
import copy
import unittest
from types import SimpleNamespace

from pylon_bridge.imu_packets import imu_from_packet


def packet():
    return {
        "type": "pylon_imu", "version": 1, "vesselId": "a" * 32,
        "universalTime": 123.5,
        "angularVelocity": [0.1, -0.2, 0.3],
        "linearAcceleration": [0, 0, 9.81],
    }


class ImuPacketTests(unittest.TestCase):
    def test_preserves_si_body_measurements(self):
        state = imu_from_packet(packet())
        self.assertEqual(state.angular_velocity, (0.1, -0.2, 0.3))
        self.assertEqual(state.linear_acceleration, (0, 0, 9.81))
        self.assertEqual(state.universal_time, 123.5)

    def test_rejects_invalid_samples(self):
        for field, value in (
            ("type", "pylon_ground_truth"), ("version", 2), ("vesselId", ""),
            ("universalTime", float("nan")),
            ("angularVelocity", [0, 0]),
            ("linearAcceleration", [0, float("inf"), 0]),
        ):
            with self.subTest(field=field):
                data = packet()
                data[field] = value
                with self.assertRaises(ValueError):
                    imu_from_packet(data)

    def test_ros_publication_and_vessel_switch(self):
        try:
            from builtin_interfaces.msg import Time
            from pylon_bridge.udp_bridge import PyLoNBridge
        except ImportError as exc:
            self.skipTest(f"ROS runtime unavailable: {exc}")
        published = []
        warnings = []
        node = SimpleNamespace(
            latest_sample_time=None,
            imu_publisher=SimpleNamespace(publish=lambda m: published.append(copy.deepcopy(m))),
            stamp_for_packet=lambda p: Time(sec=123, nanosec=500000000),
            get_logger=lambda: SimpleNamespace(warning=warnings.append),
            simulation_clock=SimpleNamespace(reset=lambda: None),
        )
        data = packet()
        node.flight = node
        SensorsService(node).publish_imu( data)
        data["vesselId"] = "b" * 32
        data["linearAcceleration"] = [0, 0, 0]
        SensorsService(node).publish_imu( data)
        self.assertEqual(len(published), 2)
        self.assertEqual(published[0].header.frame_id, "base_link")
        self.assertEqual(published[0].header.stamp.sec, 123)
        self.assertAlmostEqual(published[0].angular_velocity.y, -0.2)
        self.assertAlmostEqual(published[0].linear_acceleration.z, 9.81)
        self.assertEqual(published[1].linear_acceleration.z, 0)
        self.assertEqual(published[0].orientation_covariance[0], -1)
        self.assertTrue(all(v == 0 for v in published[0].angular_velocity_covariance))
        data["linearAcceleration"] = [float("nan"), 0, 0]
        SensorsService(node).publish_imu( data)
        self.assertEqual(len(published), 2)
        self.assertEqual(len(warnings), 1)
