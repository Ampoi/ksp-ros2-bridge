import json
import math
import unittest

from ksp_lidar_bridge.motor_packets import (
    encode_motor_command,
    motor_commands_from_point,
    motor_state_from_packet,
)


class MotorStateTests(unittest.TestCase):
    def test_converts_motor_state_in_si_units(self):
        state = motor_state_from_packet(
            {
                "type": "ksp_motor_state",
                "name": "Arm Servo #1",
                "jointType": "revolute",
                "vessel": "Mun Rover",
                "partFlightId": 42,
                "position": math.pi / 2,
                "velocity": 0.5,
                "effort": -12.0,
                "current": 1.25,
                "target": math.pi,
                "powered": True,
                "engaged": True,
                "locked": False,
            }
        )
        self.assertEqual(state.name, "arm_servo_1")
        self.assertEqual(state.joint_type, "revolute")
        self.assertEqual(state.part_flight_id, 42)
        self.assertAlmostEqual(state.position, math.pi / 2)
        self.assertEqual(state.effort, -12.0)
        self.assertEqual(state.current, 1.25)
        self.assertTrue(state.powered)

    def test_rejects_nonfinite_state(self):
        packet = {
            "type": "ksp_motor_state",
            "name": "linear_1",
            "jointType": "prismatic",
            "position": "nan",
            "velocity": 0,
            "effort": 0,
            "current": 0,
            "target": 0,
        }
        with self.assertRaisesRegex(ValueError, "position"):
            motor_state_from_packet(packet)

    def test_rejects_unknown_joint_type(self):
        packet = {
            "type": "ksp_motor_state",
            "name": "motor",
            "jointType": "planar",
        }
        with self.assertRaisesRegex(ValueError, "unsupported"):
            motor_state_from_packet(packet)


class MotorCommandTests(unittest.TestCase):
    def test_builds_position_commands_for_multiple_joints(self):
        commands = motor_commands_from_point(
            ["Servo A", "linear_b"],
            [math.pi / 2, 0.75],
            [0.4, 0.2],
            [100.0, 1500.0],
            sequence=7,
        )
        self.assertEqual([item["name"] for item in commands], ["servo_a", "linear_b"])
        self.assertEqual(commands[0]["mode"], "position")
        self.assertAlmostEqual(commands[0]["position"], math.pi / 2)
        self.assertEqual(commands[1]["velocity"], 0.2)
        self.assertEqual(commands[1]["effort"], 1500.0)
        self.assertEqual(commands[1]["sequence"], 7)

    def test_builds_velocity_only_command(self):
        command = motor_commands_from_point(
            ["servo_12"], [], [-0.5], [], sequence=1
        )[0]
        self.assertEqual(command["mode"], "velocity")
        self.assertFalse(command["hasPosition"])
        self.assertTrue(command["hasVelocity"])

    def test_rejects_mismatched_arrays(self):
        with self.assertRaisesRegex(ValueError, "positions"):
            motor_commands_from_point(
                ["servo_1", "servo_2"], [0.0], [], [], sequence=1
            )

    def test_rejects_duplicate_sanitized_names(self):
        with self.assertRaisesRegex(ValueError, "unique"):
            motor_commands_from_point(
                ["Servo A", "servo-a"], [0.0, 1.0], [], [], sequence=1
            )

    def test_encodes_compact_json(self):
        command = motor_commands_from_point(
            ["servo_1"], [1.0], [], [], sequence=3
        )[0]
        encoded = encode_motor_command(command)
        self.assertNotIn(b" ", encoded)
        self.assertEqual(json.loads(encoded)["name"], "servo_1")


if __name__ == "__main__":
    unittest.main()
