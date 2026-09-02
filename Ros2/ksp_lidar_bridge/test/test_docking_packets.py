import json
import unittest

from ksp_lidar_bridge.docking_packets import (
    ACTION_RELEASE,
    ACTION_SELECT_CAMERA,
    docking_port_manifest_from_packet,
    docking_port_state_from_packet,
    encode_docking_port_command,
)


class DockingPacketTests(unittest.TestCase):
    def test_parses_manifest(self):
        packet = {
            "type": "ksp_docking_port_manifest",
            "version": 1,
            "ports": [{"name": "Docking Port 42", "partFlightId": 42, "moduleIndex": 3}],
        }
        self.assertEqual(docking_port_manifest_from_packet(packet), ["docking_port_42"])

    def test_rejects_duplicate_manifest_names_after_normalization(self):
        packet = {
            "type": "ksp_docking_port_manifest", "version": 1,
            "ports": [
                {"name": "port A", "partFlightId": 1, "moduleIndex": 0},
                {"name": "port-a", "partFlightId": 2, "moduleIndex": 0},
            ],
        }
        with self.assertRaisesRegex(ValueError, "duplicate"):
            docking_port_manifest_from_packet(packet)

    def test_parses_state_without_partner(self):
        state = docking_port_state_from_packet({
            "type": "ksp_docking_port_state", "version": 1,
            "name": "docking_port_10_2", "partFlightId": 10, "moduleIndex": 2,
            "nodeType": "size1", "state": "Ready", "docked": False,
            "acquiring": False, "releasable": False, "cameraActive": True,
            "partnerName": "", "partnerPartFlightId": 0,
        })
        self.assertEqual(state.name, "docking_port_10_2")
        self.assertEqual(state.partner_name, "")
        self.assertTrue(state.camera_active)

    def test_encodes_typed_actions(self):
        packet = json.loads(encode_docking_port_command("Docking Port 4", ACTION_SELECT_CAMERA, 7))
        self.assertEqual(packet["name"], "docking_port_4")
        self.assertEqual(packet["action"], ACTION_SELECT_CAMERA)
        self.assertEqual(packet["sequence"], 7)
        self.assertEqual(json.loads(encode_docking_port_command("port", ACTION_RELEASE, 8))["action"], ACTION_RELEASE)

    def test_rejects_unknown_action_and_invalid_sequence(self):
        with self.assertRaisesRegex(ValueError, "unsupported"):
            encode_docking_port_command("port", 99, 1)
        with self.assertRaisesRegex(ValueError, "sequence"):
            encode_docking_port_command("port", ACTION_RELEASE, 0)
        with self.assertRaisesRegex(ValueError, "name"):
            encode_docking_port_command("", ACTION_RELEASE, 1)


if __name__ == "__main__":
    unittest.main()
