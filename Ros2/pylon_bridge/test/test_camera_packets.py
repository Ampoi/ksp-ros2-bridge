import base64
import hashlib
import unittest

from pylon_bridge.camera_packets import (
    CameraFrameAssembler,
    camera_optical_rotation,
    camera_topics,
)


class CameraFrameAssemblerTests(unittest.TestCase):
    def chunks(self, payload=b"abcdefghijkl", chunk_size=5):
        checksum = hashlib.sha256(payload).hexdigest()
        pieces = [payload[index : index + chunk_size] for index in range(0, len(payload), chunk_size)]
        packets = []
        for index, piece in enumerate(pieces):
            packets.append(
                {
                    "type": "pylon_camera_frame_chunk",
                    "version": 1,
                    "sessionId": "a" * 32,
                    "sequence": 7,
                    "chunkIndex": index,
                    "chunkCount": len(pieces),
                    "frameBytes": len(payload),
                    "sha256": checksum,
                    "sensorId": "Front Camera",
                    "partFlightId": 42,
                    "universalTime": 123.5,
                    "width": 2,
                    "height": 2,
                    "step": 6,
                    "encoding": "rgb8",
                    "verticalFovDeg": 60,
                    "framePosition": [1, 2, 3],
                    "frameRotation": [0, 0, 0, 2],
                    "data": base64.b64encode(piece).decode("ascii"),
                }
            )
        return packets

    def test_reassembles_out_of_order_rgb_frame(self):
        assembler = CameraFrameAssembler()
        packets = self.chunks()
        self.assertIsNone(assembler.consume(packets[2], now=1.0))
        self.assertIsNone(assembler.consume(packets[0], now=1.1))
        frame = assembler.consume(packets[1], now=1.2)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.data, b"abcdefghijkl")
        self.assertEqual(frame.encoding, "rgb8")
        self.assertEqual(frame.sensor_id, "Front Camera")
        self.assertEqual(frame.source, "rgb_camera")
        self.assertEqual(frame.frame_rotation, (0.0, 0.0, 0.0, 1.0))

    def test_preserves_docking_camera_source(self):
        assembler = CameraFrameAssembler()
        packets = self.chunks()
        for packet in packets:
            packet["source"] = "docking_port"
        frame = None
        for packet in packets:
            frame = assembler.consume(packet) or frame
        self.assertIsNotNone(frame)
        self.assertEqual(frame.source, "docking_port")

    def test_prefers_explicit_sensor_id(self):
        assembler = CameraFrameAssembler()
        packets = self.chunks()
        for packet in packets:
            packet["sensorId"] = "camera_ab12"
        frame = None
        for packet in packets:
            frame = assembler.consume(packet) or frame
        self.assertIsNotNone(frame)
        self.assertEqual(frame.sensor_id, "camera_ab12")

    def test_rejects_checksum_mismatch(self):
        assembler = CameraFrameAssembler()
        packets = self.chunks()
        packets[-1]["data"] = base64.b64encode(b"zz").decode("ascii")
        assembler.consume(packets[0])
        assembler.consume(packets[1])
        with self.assertRaisesRegex(ValueError, "checksum"):
            assembler.consume(packets[2])

    def test_expires_incomplete_frame(self):
        assembler = CameraFrameAssembler(timeout_sec=1.0)
        assembler.consume(self.chunks()[0], now=2.0)
        self.assertEqual(assembler.pending_count, 1)
        assembler.expire(now=3.0)
        self.assertEqual(assembler.pending_count, 0)

    def test_rejects_non_rgb_layout(self):
        assembler = CameraFrameAssembler()
        packet = self.chunks()[0]
        packet["encoding"] = "rgba8"
        with self.assertRaisesRegex(ValueError, "rgb8"):
            assembler.consume(packet)


class CameraTopicTests(unittest.TestCase):
    def test_builds_standard_camera_topics(self):
        self.assertEqual(
            camera_topics("Front Camera"),
            (
                "/ksp_vessel/camera/front_camera/image_raw",
                "/ksp_vessel/camera/front_camera/camera_info",
            ),
        )

    def test_routes_docking_camera_under_docking_namespace(self):
        self.assertEqual(
            camera_topics("Dock Port 1", "/robot", "docking_port"),
            (
                "/robot/docking_ports/dock_port_1/camera/image_raw",
                "/robot/docking_ports/dock_port_1/camera/camera_info",
            ),
        )

    def test_uses_explicit_docking_prefix(self):
        self.assertEqual(
            camera_topics("port", "/robot", "docking_port", "/custom/docks"),
            (
                "/custom/docks/port/camera/image_raw",
                "/custom/docks/port/camera/camera_info",
            ),
        )

    def test_rotates_sensor_axes_into_camera_optical_convention(self):
        self.assertEqual(
            camera_optical_rotation((0.0, 0.0, 0.0, 1.0)),
            (-0.5, 0.5, -0.5, 0.5),
        )


if __name__ == "__main__":
    unittest.main()
