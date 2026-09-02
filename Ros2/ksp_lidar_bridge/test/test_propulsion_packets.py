import json
import unittest

from ksp_lidar_bridge.propulsion_packets import (
    encode_propulsion_command,
    main_throttle_command,
    propulsion_command_from_json,
    propulsion_state_json,
    rcs_command,
)


class PropulsionCommandTests(unittest.TestCase):
    def test_builds_batch_module_command(self):
        packet = propulsion_command_from_json(
            '{"commands":[{"name":"Main Engine #1","kind":"engine",'
            '"enabled":true,"throttle":0.65}],"timeout":0.4}',
            sequence=7,
        )
        command = packet["commands"][0]
        self.assertEqual(command["name"], "main_engine_1")
        self.assertTrue(command["hasEnabled"])
        self.assertTrue(command["enabled"])
        self.assertTrue(command["hasThrottle"])
        self.assertEqual(command["throttle"], 0.65)
        self.assertEqual(packet["timeoutSeconds"], 0.4)
        self.assertEqual(packet["sequence"], 7)

    def test_supports_all_modules_release(self):
        packet = propulsion_command_from_json(
            '{"commands":[{"name":"*","release":true}]}', sequence=2
        )
        self.assertEqual(packet["commands"][0]["name"], "*")
        self.assertTrue(packet["commands"][0]["release"])

    def test_builds_main_throttle_and_rcs_commands(self):
        throttle = main_throttle_command(0.8, sequence=3)
        self.assertTrue(throttle["hasMainThrottle"])
        self.assertEqual(throttle["mainThrottle"], 0.8)

        rcs = rcs_command(0.1, -0.2, 0.3, -0.4, 0.5, -0.6, sequence=4)
        self.assertTrue(rcs["hasRcsCommand"])
        self.assertEqual(rcs["rcsY"], -0.2)
        self.assertEqual(rcs["rcsRoll"], -0.6)

    def test_rejects_out_of_range_values(self):
        with self.assertRaisesRegex(ValueError, "between 0 and 1"):
            propulsion_command_from_json(
                '{"commands":[{"name":"engine_1_0","throttle":1.1}]}',
                sequence=1,
            )
        with self.assertRaisesRegex(ValueError, "between -1 and 1"):
            propulsion_command_from_json('{"rcs":{"roll":-2}}', sequence=1)

    def test_encodes_compact_json(self):
        encoded = encode_propulsion_command(main_throttle_command(0.5, sequence=5))
        self.assertNotIn(b" ", encoded)
        self.assertEqual(json.loads(encoded)["type"], "ksp_propulsion_command")


class PropulsionStateTests(unittest.TestCase):
    def test_normalizes_engine_state(self):
        state = propulsion_state_json(
            {
                "type": "ksp_propulsion_state",
                "version": 1,
                "name": "Engine 42 1",
                "kind": "engine",
                "partFlightId": 42,
                "moduleIndex": 1,
                "throttle": 0.4,
                "throttleLimit": 0.75,
                "thrust": 12.5,
                "maxThrust": 20.0,
            }
        )
        decoded = json.loads(state)
        self.assertEqual(decoded["name"], "engine_42_1")
        self.assertEqual(decoded["thrust"], 12.5)

    def test_rejects_unknown_kind(self):
        with self.assertRaisesRegex(ValueError, "unsupported"):
            propulsion_state_json(
                {"type": "ksp_propulsion_state", "version": 1, "kind": "warp"}
            )


if __name__ == "__main__":
    unittest.main()
