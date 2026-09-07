import unittest

from ksp_lidar_bridge.vehicle_packets import (
    actuator_command,
    actuator_manifest_from_packet,
    actuator_names_to_remove,
    actuator_state_from_packet,
    body_wrench_command,
    control_authority_command,
    ground_truth_from_packet,
    nearby_vessels_from_packet,
)


class GroundTruthPacketTests(unittest.TestCase):
    def test_parses_and_normalizes_ground_truth(self):
        state = ground_truth_from_packet(
            {
                "type": "ksp_ground_truth",
                "version": 1,
                "vesselId": "abc",
                "originSequence": 2,
                "position": [1, 2, 3],
                "rotation": [0, 0, 0, 2],
                "linearVelocity": [4, 5, 6],
                "angularVelocity": [0.1, 0.2, 0.3],
                "linearVelocityBody": [6, 5, 4],
                "angularVelocityBody": [0.3, 0.2, 0.1],
                "linearAcceleration": [0, 0, -9.81],
                "angularAcceleration": [0, 0, 0],
            }
        )
        self.assertEqual(state.position, (1.0, 2.0, 3.0))
        self.assertEqual(state.rotation, (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(state.origin_sequence, 2)
        self.assertEqual(state.linear_velocity_body, (6.0, 5.0, 4.0))
        self.assertEqual(state.angular_velocity_body, (0.3, 0.2, 0.1))

    def test_rejects_nonfinite_ground_truth(self):
        packet = {
            "type": "ksp_ground_truth", "version": 1,
            "position": [0, 0, "nan"], "rotation": [0, 0, 0, 1],
            "linearVelocity": [0, 0, 0], "angularVelocity": [0, 0, 0],
            "linearAcceleration": [0, 0, 0], "angularAcceleration": [0, 0, 0],
        }
        with self.assertRaisesRegex(ValueError, "position"):
            ground_truth_from_packet(packet)


class VehicleCommandTests(unittest.TestCase):
    def test_builds_body_wrench(self):
        command = body_wrench_command(
            (1, 2, 3), (4, 5, 6), 7, 0.5, "vessel", "controller", "lease"
        )
        self.assertEqual(command["frame"], "base_link")
        self.assertEqual(command["force"], [1, 2, 3])
        self.assertEqual(command["version"], 2)
        self.assertEqual(command["vesselId"], "vessel")

    def test_builds_control_authority_command(self):
        command = control_authority_command(
            "acquire", "vessel", "controller", "lease", 100, 2.0, True, 8
        )
        self.assertEqual(command["action"], "acquire")
        self.assertEqual(command["priority"], 100)
        self.assertTrue(command["suppressSas"])

    def test_builds_typed_actuator_command(self):
        command = actuator_command(
            "wheel", "Wheel #1",
            {"enabled": True, "targetAngularVelocity": 2.0, "timeoutSeconds": 0.4},
            3,
            "vessel", "controller", "lease",
        )
        self.assertEqual(command["name"], "wheel_1")
        self.assertEqual(command["targetAngularVelocity"], 2.0)

    def test_validates_actuator_state(self):
        state = actuator_state_from_packet(
            {
                "type": "ksp_actuator_state", "version": 1,
                "actuatorType": "engine", "name": "Main Engine",
                "throttle": 0.5, "thrust": 12.0, "maxThrust": 20.0,
            }
        )
        self.assertEqual(state["name"], "main_engine")
        self.assertEqual(state["thrust"], 12.0)

    def test_validates_separation_state(self):
        state = actuator_state_from_packet(
            {
                "type": "ksp_actuator_state", "version": 1,
                "actuatorType": "separation", "name": "Decoupler #1",
                "mechanism": "decoupler", "available": False,
                "separated": True,
            }
        )
        self.assertEqual(state["name"], "decoupler_1")
        self.assertEqual(state["mechanism"], "decoupler")
        self.assertTrue(state["separated"])

    def test_rejects_unknown_separation_mechanism(self):
        with self.assertRaisesRegex(ValueError, "separation mechanism"):
            actuator_state_from_packet(
                {
                    "type": "ksp_actuator_state", "version": 1,
                    "actuatorType": "separation", "name": "port_1",
                    "mechanism": "docking", "available": True,
                    "separated": False,
                }
            )

    def test_validates_actuator_manifest(self):
        manifest = actuator_manifest_from_packet(
            {
                "type": "ksp_actuator_manifest",
                "version": 1,
                "actuators": [
                    {"actuatorType": "wheel", "name": "Wheel #1"},
                    {"actuatorType": "motor", "name": "Servo 2"},
                    {"actuatorType": "separation", "name": "Decoupler 3"},
                ],
            }
        )
        self.assertEqual(
            manifest,
            {
                "wheel_1": "wheel",
                "servo_2": "motor",
                "decoupler_3": "separation",
            },
        )

    def test_builds_separation_command(self):
        command = actuator_command(
            "separation", "Decoupler #1", {"separate": True}, 9,
            "vessel", "controller", "lease"
        )
        self.assertEqual(command["actuatorType"], "separation")
        self.assertEqual(command["name"], "decoupler_1")
        self.assertIs(command["separate"], True)

    def test_preserves_false_separation_command_as_no_op(self):
        command = actuator_command(
            "separation", "fairing_12_0", {"separate": False}, 10,
            "vessel", "controller", "lease"
        )
        self.assertIs(command["separate"], False)

    def test_rejects_duplicate_manifest_names_after_normalization(self):
        packet = {
            "type": "ksp_actuator_manifest",
            "version": 1,
            "actuators": [
                {"actuatorType": "wheel", "name": "Wheel #1"},
                {"actuatorType": "wheel", "name": "wheel-1"},
            ],
        }
        with self.assertRaisesRegex(ValueError, "duplicate"):
            actuator_manifest_from_packet(packet)

    def test_retains_separated_actuator_until_vessel_changes(self):
        current = {"decoupler_1": "separation", "wheel_2": "wheel"}
        self.assertEqual(
            actuator_names_to_remove({}, current, {"decoupler_1"}, False),
            ["wheel_2"],
        )
        self.assertEqual(
            actuator_names_to_remove({}, current, {"decoupler_1"}, True),
            ["decoupler_1", "wheel_2"],
        )


if __name__ == "__main__":
    unittest.main()


class NearbyTruthTests(unittest.TestCase):
    def packet(self):
        return dict(type="ksp_nearby_vessels", version=1, vesselId="observer",
            originSequence=7, universalTime=123., position=[1e8, 0., 0.],
            linearVelocity=[2200., 0., 0.], vessels=[dict(vesselId="debris", vessel="debris",
            isDebris=True, position=[1e8 + 15.125, 0., 0.], linearVelocity=[2200.25, 0., 0.])])

    def test_shared_absolute_origin_preserves_submetre_relative_motion(self):
        result = nearby_vessels_from_packet(self.packet())
        self.assertEqual(result.vessels[0].position[0] - result.observer_position[0], 15.125)
        self.assertEqual(result.vessels[0].linear_velocity[0] - result.observer_linear_velocity[0], .25)
        self.assertEqual(result.origin_sequence, 7)

    def test_rejects_duplicate_self_and_nonfinite_states(self):
        for invalid in ("self", "duplicate", "nonfinite", "boolean"):
            packet = self.packet()
            if invalid == "self": packet["vessels"][0]["vesselId"] = "observer"
            if invalid == "duplicate": packet["vessels"] *= 2
            if invalid == "nonfinite": packet["vessels"][0]["position"][0] = float("nan")
            if invalid == "boolean": packet["vessels"][0]["isDebris"] = "false"
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                nearby_vessels_from_packet(packet)

    def test_empty_snapshot_reports_disappearance(self):
        packet = self.packet()
        packet["vessels"] = []
        self.assertEqual(nearby_vessels_from_packet(packet).vessels, ())
