import unittest
from pylon_bridge.domain.session import SessionTracker
from pylon_bridge.packet_conversion import decode_datagram

def packet(generation=1,instance='one',**kw):
    return dict(type='pylon_session',runtimeInstance=instance,runtimeGeneration=generation,
                runtimeEpoch=f'epoch{generation}',runtimeVesselId='vessel',vesselId='vessel',
                universalTime=float(generation),available=True,**kw)

class SessionTests(unittest.TestCase):
    def test_heartbeat_only_and_old_generation_rejection(self):
        tracker=SessionTracker();old=packet();new=packet(2)
        self.assertFalse(tracker.accepts(old))
        self.assertTrue(tracker.observe(old,1))
        self.assertTrue(tracker.observe(new,2))
        self.assertFalse(tracker.observe(old,3))
        self.assertFalse(tracker.accepts(old))
        self.assertTrue(tracker.accepts(new))
    def test_restart_retires_previous_instance(self):
        tracker=SessionTracker();tracker.observe(packet(10),1);tracker.observe(packet(1,'two'),2)
        self.assertFalse(tracker.observe(packet(11),3))
        self.assertEqual(tracker.key.instance,'two')
    def test_unavailable_stale_and_wrong_vessel_commands(self):
        tracker=SessionTracker();tracker.observe(packet(),1)
        with self.assertRaises(ValueError):tracker.command_fields(5,3)
        self.assertFalse(tracker.accepts(dict(packet(),vesselId='wrong')))
        tracker.observe(dict(packet(),available=False),2)
        with self.assertRaises(ValueError):tracker.command_fields(2,3)
    def test_old_wire_protocol_rejected(self):
        for data in [b'{"type":"ksp_lidar_scan","version":1}',b'{"type":"pylon_imu","version":0}',b'{"type":"pylon_imu","version":true}']:
            with self.assertRaises(ValueError):decode_datagram(data)
    def test_delayed_heartbeat_does_not_rebase_session(self):
        tracker=SessionTracker();tracker.observe(dict(packet(),universalTime=10),1)
        self.assertFalse(tracker.observe(dict(packet(),universalTime=9),2))
        self.assertEqual(tracker.last_seen,1)

    def test_bridge_restart_changes_ros_generation_even_for_the_same_runtime(self):
        first=SessionTracker();second=SessionTracker()
        first.observe(packet(),1);second.observe(packet(),1)
        self.assertEqual(first.key,second.key)
        self.assertNotEqual(first.generation,second.generation)
