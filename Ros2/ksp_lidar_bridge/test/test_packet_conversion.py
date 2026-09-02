import math
import unittest

from ksp_lidar_bridge.packet_conversion import (
    chunked_vectors,
    decode_datagram,
    expired_topic_names,
    fibonacci_hemisphere_directions,
    laser_scan_from_packet,
    lidar_topic_from_packet,
    lidar_topic_suffix,
    normalized_ranges,
    packet_lidar_name,
    packet_part_name,
    points_from_packet,
    sanitize_ros_name,
    sensor_pose_from_packet,
)


class DatagramTests(unittest.TestCase):
    def test_decodes_json_object(self):
        self.assertEqual(
            decode_datagram(b'{"type":"ksp_lidar_scan"}'),
            {"type": "ksp_lidar_scan"},
        )

    def test_rejects_non_object_json(self):
        with self.assertRaisesRegex(ValueError, "JSON object"):
            decode_datagram(b"[]")


class VectorTests(unittest.TestCase):
    def test_reads_flat_vectors(self):
        self.assertEqual(
            list(chunked_vectors([1, 2, 3, 4, 5, 6])),
            [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)],
        )

    def test_reads_legacy_nested_vectors(self):
        self.assertEqual(
            list(chunked_vectors([[1, 2, 3], [4, 5, 6]])),
            [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)],
        )

    def test_keeps_alignment_when_nested_vectors_start_with_null(self):
        vectors = list(chunked_vectors([None, [4, 5, 6]]))
        self.assertTrue(all(math.isnan(axis) for axis in vectors[0]))
        self.assertEqual(vectors[1], (4.0, 5.0, 6.0))

    def test_marks_incomplete_flat_vector_invalid(self):
        vectors = list(chunked_vectors([1, 2, 3, 4]))
        self.assertEqual(vectors[0], (1.0, 2.0, 3.0))
        self.assertTrue(all(math.isnan(axis) for axis in vectors[1]))


class PointConversionTests(unittest.TestCase):
    def packet(self, **overrides):
        packet = {
            "rayCount": 3,
            "maxDistance": 10,
            "ranges": [2, -1, 4],
        }
        packet.update(overrides)
        return packet

    def test_uses_flat_hit_points_from_current_ksp_packet(self):
        packet = self.packet(points=[1, 2, 3, None, None, None, 7, 8, 9])
        self.assertEqual(points_from_packet(packet), [(1.0, 2.0, 3.0), (7.0, 8.0, 9.0)])

    def test_uses_legacy_nested_hit_points(self):
        packet = self.packet(points=[[1, 2, 3], [None, None, None], [7, 8, 9]])
        self.assertEqual(points_from_packet(packet), [(1.0, 2.0, 3.0), (7.0, 8.0, 9.0)])

    def test_calculates_points_from_flat_directions(self):
        packet = self.packet(points=[], directions=[1, 0, 0, 0, 1, 0, 0, 0, -1])
        self.assertEqual(points_from_packet(packet), [(2.0, 0.0, 0.0), (0.0, 0.0, -4.0)])

    def test_calculates_points_from_legacy_nested_directions(self):
        packet = self.packet(directions=[[1, 0, 0], [0, 1, 0], [0, 0, -1]])
        self.assertEqual(points_from_packet(packet), [(2.0, 0.0, 0.0), (0.0, 0.0, -4.0)])

    def test_reconstructs_compact_fibonacci_hemisphere_points(self):
        packet = self.packet(
            coordinateFrame="ros_sensor",
            layout="fibonacci-hemisphere",
        )
        points = points_from_packet(packet)
        expected = fibonacci_hemisphere_directions(3)
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0][0], expected[0][0] * 2)
        self.assertAlmostEqual(points[0][1], expected[0][1] * 2)
        self.assertAlmostEqual(points[0][2], expected[0][2] * 2)
        self.assertAlmostEqual(points[1][0], expected[2][0] * 4)
        self.assertAlmostEqual(points[1][1], expected[2][1] * 4)
        self.assertAlmostEqual(points[1][2], expected[2][2] * 4)

    def test_single_fibonacci_ray_points_forward(self):
        self.assertEqual(fibonacci_hemisphere_directions(1), [(1.0, 0.0, 0.0)])


class SensorPoseTests(unittest.TestCase):
    def test_reads_and_normalizes_ros_sensor_pose(self):
        pose = sensor_pose_from_packet(
            {
                "coordinateFrame": "ros_sensor",
                "framePosition": [1, 2, 3],
                "frameRotation": [0, 0, 0, 2],
            }
        )
        self.assertIsNotNone(pose)
        self.assertEqual(pose.translation, (1.0, 2.0, 3.0))
        self.assertEqual(pose.rotation, (0.0, 0.0, 0.0, 1.0))

    def test_rejects_missing_or_invalid_sensor_pose(self):
        self.assertIsNone(sensor_pose_from_packet({}))
        self.assertIsNone(
            sensor_pose_from_packet(
                {
                    "coordinateFrame": "ros_sensor",
                    "framePosition": [0, 0, 0],
                    "frameRotation": [0, 0, 0, 0],
                }
            )
        )

class ScanNormalizationTests(unittest.TestCase):
    def test_normalizes_ranges_to_requested_count(self):
        ranges = normalized_ranges([1, -1, 11, "4"], 5, 10.0)
        self.assertEqual(ranges[0], 1.0)
        self.assertTrue(math.isinf(ranges[1]))
        self.assertTrue(math.isinf(ranges[2]))
        self.assertEqual(ranges[3], 4.0)
        self.assertTrue(math.isinf(ranges[4]))

    def test_rejects_overflowing_range(self):
        ranges = normalized_ranges([10**400], 1, 10.0)
        self.assertTrue(math.isinf(ranges[0]))

    def test_builds_full_circle_scan_values(self):
        scan = laser_scan_from_packet(
            {
                "horizontalCount": 4,
                "horizontalFovDeg": 360,
                "maxDistance": 20,
                "scanRateHz": 2,
                "ranges": [1, 2, 3, 4],
            }
        )
        self.assertAlmostEqual(scan.angle_min, -math.pi)
        self.assertAlmostEqual(scan.angle_increment, math.pi / 2)
        self.assertAlmostEqual(scan.angle_max, math.pi / 2)
        self.assertEqual(scan.scan_time, 0.5)
        self.assertEqual(scan.time_increment, 0.125)
        self.assertEqual(scan.ranges, [1.0, 2.0, 3.0, 4.0])


class NameTests(unittest.TestCase):
    def test_prefers_part_name(self):
        packet = {
            "partName": "roof_lidar",
            "lidarName": "legacy_name",
            "name": "oldest_name",
        }
        self.assertEqual(packet_part_name(packet), "roof_lidar")

    def test_sanitizes_explicit_lidar_name(self):
        packet = {"lidarName": "Front LiDAR #1"}
        self.assertEqual(sanitize_ros_name(packet_lidar_name(packet)), "front_lidar_1")

    def test_builds_name_from_vessel_and_part(self):
        packet = {"vessel": "Mun Rover", "partFlightId": "42"}
        self.assertEqual(packet_part_name(packet), "mun_rover_42")

    def test_builds_topic_suffix_for_each_lidar_mode(self):
        self.assertEqual(lidar_topic_suffix("2D"), "lidar/scan")
        self.assertEqual(lidar_topic_suffix("3d"), "lidar/points")

    def test_builds_full_topic_from_part_name_and_mode(self):
        self.assertEqual(
            lidar_topic_from_packet({"partName": "Front LiDAR", "mode": "2D"}),
            "/ros2_ksp/front_lidar/lidar/scan",
        )
        self.assertEqual(
            lidar_topic_from_packet(
                {"partName": "roof_lidar", "mode": "3D"},
                "/robot/sensors/",
            ),
            "/robot/sensors/roof_lidar/lidar/points",
        )

    def test_rejects_unknown_lidar_mode(self):
        with self.assertRaisesRegex(ValueError, "unsupported"):
            lidar_topic_suffix("camera")


class TopicLifetimeTests(unittest.TestCase):
    def test_expires_only_topics_at_or_beyond_timeout(self):
        last_seen = {"/fresh": 8.1, "/boundary": 7.0, "/stale": 1.0}
        self.assertEqual(
            expired_topic_names(last_seen, now=10.0, timeout=3.0),
            ["/boundary", "/stale"],
        )


if __name__ == "__main__":
    unittest.main()
