import copy
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from pylon_bridge.star_tracker_packets import star_tracker_from_packet


def packet(**changes):
    data = dict(type='pylon_star_tracker', version=1, vesselId='a'*32,
                sessionId='b'*32, sensorId='star_tracker_port', sequence=1,
                universalTime=42.25, referenceFrame='kerbol_inertial', measuredFrame='base_link',
                valid=True, reason='tracking', orientation=[0, 0, 0, 1],
                noiseStdRad=.0001, angularRateDegSec=.2)
    data.update(changes)
    return data


class PacketTests(unittest.TestCase):
    def test_valid_and_every_loss_state(self):
        from pylon_bridge.star_tracker_packets import REASONS
        self.assertEqual(star_tracker_from_packet(packet()).orientation, (0, 0, 0, 1))
        for reason in REASONS - {'tracking'}:
            state = star_tracker_from_packet(packet(valid=False, reason=reason, orientation=None))
            self.assertFalse(state.valid)
            self.assertIsNone(state.orientation)

    def test_rejects_corruption_and_false_validity(self):
        for changes in [dict(valid=False), dict(reason='acquiring'), dict(valid='false'),
                        dict(orientation=[0, 0, 0, 0]), dict(orientation=[0, 0, 0, 2]),
                        dict(orientation=[float('nan'), 0, 0, 1]), dict(sequence=0),
                        dict(sequence=True), dict(version=True), dict(vesselId='bad'),
                        dict(sensorId='../bad'), dict(sensorId='a'*65), dict(noiseStdRad=0),
                        dict(noiseStdRad=True), dict(angularRateDegSec=-1),
                        dict(universalTime=float('inf')), dict(referenceFrame='map'),
                        dict(measuredFrame='camera'), dict(reason=[]), dict(orientation=[1, 2])]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                star_tracker_from_packet(packet(**changes))


class BridgeTests(unittest.TestCase):
    def setUp(self):
        from builtin_interfaces.msg import Time
        from pylon_bridge.services.star_tracker import StarTrackerService
        self.published = {}
        self.removed = []
        self.node = SimpleNamespace()
        self.node.star_tracker = StarTrackerService(self.node)
        self.node.topic_prefix = '/test'
        self.node.args = SimpleNamespace(topic_timeout_sec=2.)
        self.node.active_vessel_id = 'a'*32
        self.node.stamp_for_packet = lambda p: Time(sec=42, nanosec=250000000)
        self.node.get_logger = lambda: SimpleNamespace(warning=lambda m: None)
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=50)))
        self.node.publisher_for = lambda topic, *args: SimpleNamespace(
            publish=lambda msg: self.published.setdefault(topic, []).append(copy.deepcopy(msg)))
        self.node.remove_publisher = lambda topic, reason: self.removed.append(topic)
        self.node.flight = self.node
        self.node.sensors = self.node
        self.prefix = '/test/star_tracker/star_tracker_port'

    def test_loss_never_publishes_attitude_or_reuses_last_quaternion(self):
        self.node.star_tracker.publish_star_tracker(packet())
        self.node.star_tracker.publish_star_tracker(packet(sequence=2, valid=False, reason='sun_exclusion', orientation=None))
        self.node.star_tracker.publish_star_tracker(packet())  # delayed good packet
        states = self.published[self.prefix + '/state']
        self.assertEqual(len(states), 2)
        self.assertEqual(len(self.published[self.prefix + '/attitude']), 1)
        self.assertEqual(states[0].header.frame_id, 'kerbol_inertial')
        self.assertEqual(states[0].measured_frame_id, 'base_link')
        self.assertAlmostEqual(states[0].orientation_covariance[4], 1e-8)
        self.assertFalse(states[1].valid)
        self.assertEqual(states[1].orientation.w, 0.)
        self.assertEqual(states[1].orientation_covariance[0], -1.)
        self.assertEqual(states[1].reason, 'sun_exclusion')
        self.assertEqual(states[0].header.stamp.nanosec, 250000000)

    def test_timeout_heartbeat_recovery_and_bounded_cleanup(self):
        with patch('pylon_bridge.services.star_tracker.time.monotonic', return_value=10.):
            self.node.star_tracker.publish_star_tracker(packet())
        with patch('pylon_bridge.services.star_tracker.time.monotonic', return_value=13.):
            self.node.star_tracker.expire_star_trackers()
            self.node.star_tracker.expire_star_trackers()
            self.node.star_tracker.publish_star_tracker(packet(sequence=2))
        states = self.published[self.prefix + '/state']
        self.assertEqual([s.reason for s in states], ['tracking', 'stale', 'stale', 'tracking'])
        self.assertEqual(len(self.published[self.prefix + '/attitude']), 2)
        with patch('pylon_bridge.services.star_tracker.time.monotonic', return_value=50.):
            self.node.star_tracker.expire_star_trackers()
        self.assertFalse(self.node.star_trackers)
        self.assertEqual(len(self.removed), 2)

    def test_vessel_switch_and_independent_sensors(self):
        self.node.star_tracker.publish_star_tracker(packet())
        self.node.star_tracker.publish_star_tracker(packet(sensorId='star_tracker_2', sequence=8))
        self.assertEqual(len(self.node.star_trackers), 2)
        self.node.active_vessel_id = 'c'*32
        self.node.star_tracker.expire_star_trackers()
        self.assertEqual(self.published[self.prefix + '/state'][-1].reason, 'inactive')
        self.node.star_tracker.publish_star_tracker(packet(sequence=3))
        self.assertEqual(self.published[self.prefix + '/state'][-1].reason, 'inactive')
        self.node.star_tracker.publish_star_tracker(packet(vesselId='c'*32, sessionId='d'*32))
        self.assertEqual(self.published[self.prefix + '/state'][-1].vessel_id, 'c'*32)

    def test_real_ros_serialization_roundtrip(self):
        from rclpy.serialization import serialize_message, deserialize_message
        from pylon_interfaces.msg import StarTrackerState
        self.node.star_tracker.publish_star_tracker(packet())
        original = self.published[self.prefix + '/state'][0]
        decoded = deserialize_message(serialize_message(original), StarTrackerState)
        self.assertEqual(decoded, original)


class UdpRosTests(unittest.TestCase):
    def test_udp_to_dds_valid_loss_and_no_attitude_on_loss(self):
        import json
        import socket
        import time
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from geometry_msgs.msg import QuaternionStamped
        from pylon_interfaces.msg import StarTrackerState
        from pylon_bridge.udp_bridge import PyLoNBridge, parse_args
        rclpy.init(domain_id=175)
        bridge = PyLoNBridge(parse_args(['--host', '127.0.0.1', '--port', '0', '--disable-ground-truth']))
        listener = rclpy.create_node('star_tracker_test_subscriber')
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(listener)
        states, attitudes = [], []
        prefix = '/ksp_vessel/star_tracker/star_tracker_port'
        listener.create_subscription(StarTrackerState, prefix + '/state', states.append, 10)
        listener.create_subscription(QuaternionStamped, prefix + '/attitude', attitudes.append, 10)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        endpoint = bridge.sock.getsockname()
        session = dict(type='pylon_session',version=1,runtimeInstance='test',runtimeGeneration=1,
                       runtimeEpoch='epoch',runtimeVesselId='a'*32,vesselId='a'*32,
                       vesselName='test',available=True,universalTime=1.)
        fields = {k:v for k,v in session.items() if k.startswith('runtime')}
        try:
            seq = 0
            deadline = time.monotonic() + 5
            while not attitudes and time.monotonic() < deadline:
                seq += 1
                sender.sendto(json.dumps(session).encode(), endpoint)
                sender.sendto(json.dumps(dict(packet(sequence=seq), **fields)).encode(), endpoint)
                executor.spin_once(timeout_sec=.05)
            self.assertTrue(attitudes, 'no valid orientation received through DDS')
            self.assertTrue(any(s.valid for s in states))
            # Finish queued valid callbacks before observing the loss transition.
            for _ in range(20):
                executor.spin_once(timeout_sec=.01)
            before = len(attitudes)
            seq += 1
            sender.sendto(json.dumps(dict(packet(sequence=seq, valid=False, reason='occluded', orientation=None), **fields)).encode(), endpoint)
            deadline = time.monotonic() + 3
            while not any(s.reason == 'occluded' for s in states) and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
            self.assertTrue(any(not s.valid and s.reason == 'occluded' for s in states))
            self.assertEqual(len(attitudes), before)
        finally:
            sender.close()
            bridge.sock.close()
            executor.shutdown()
            listener.destroy_node()
            bridge.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
