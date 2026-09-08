import json
import math
import unittest

from pylon_bridge.motor_packets import (
    encode_motor_command,

    motor_state_from_packet,
)


class MotorStateTests(unittest.TestCase):
    def test_converts_motor_state_in_si_units(self):
        state = motor_state_from_packet(
            {
                "type": "pylon_motor_state",
                "name": "Arm Servo #1",
                "jointType": "revolute",
                "vessel": "Mun Rover",
                "partFlightId": 42,
                "position": math.pi / 2,
                "velocity": 0.5,
                "effort": -12.0,
                "current": 1.25,
                "target": math.pi,
                "commandMode": "velocity",
                "commandActive": True,
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
        self.assertEqual(state.command_mode, "velocity")
        self.assertTrue(state.command_active)
        self.assertTrue(state.powered)

    def test_rejects_nonfinite_state(self):
        packet = {
            "type": "pylon_motor_state",
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
            "type": "pylon_motor_state",
            "name": "motor",
            "jointType": "planar",
        }
        with self.assertRaisesRegex(ValueError, "unsupported"):
            motor_state_from_packet(packet)


class MotorCommandTests(unittest.TestCase):
    def test_encodes_typed_command(self):
        command = dict(type='pylon_motor_command', version=1, name='servo',
                       vesselId='vessel', controllerId='controller', leaseId='lease', sequence=3)
        self.assertEqual(json.loads(encode_motor_command(command)), command)

    def test_rejects_nan_in_command(self):
        with self.assertRaises(ValueError):
            encode_motor_command(dict(position=float('nan')))
