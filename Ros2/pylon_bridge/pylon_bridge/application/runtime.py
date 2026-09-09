"""Own flight-scoped communication state independently of its presentation."""

from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional

from ..camera_packets import CameraFrameAssembler
from ..domain.session import SessionTracker
from ..domain.time_alignment import SimulationClock
from ..protocol import SESSION_PACKET_TYPE, decode_datagram, encode_datagram
from .model_transfer import ModelTransfer


@dataclass(frozen=True)
class ReceivedPacket:
    packet: Dict[str, Any]
    session_changed: bool = False


class BridgeRuntime:
    def __init__(self):
        self.session = SessionTracker()
        self.simulation_clock = SimulationClock()
        self.session_timed_out = False
        self.reset_flight_state()

    def reset_flight_state(self) -> None:
        self.simulation_clock.reset()
        self.latest_sample_time = None
        self.camera_assembler = CameraFrameAssembler()
        self.model_transfer = ModelTransfer()

    def observe_session(self, packet: Dict[str, Any], now: float) -> Optional[ReceivedPacket]:
        previous = self.session.revision
        if not self.session.observe(packet, now):
            return None
        changed = previous != self.session.revision
        if changed:
            self.reset_flight_state()
        self.session_timed_out = False
        self.note_sample_time(packet["universalTime"])
        return ReceivedPacket(packet, session_changed=changed)

    def receive(self, data: bytes, now: float) -> Optional[ReceivedPacket]:
        packet = decode_datagram(data)
        if packet["type"] == SESSION_PACKET_TYPE:
            return self.observe_session(packet, now)
        if self.session.accepts(packet):
            return ReceivedPacket(packet)
        return None

    def expire_session(self, now: float, timeout: float) -> bool:
        if (self.session_timed_out or self.session.last_seen <= 0.0
                or now - self.session.last_seen <= timeout):
            return False
        self.reset_flight_state()
        self.session.available = False
        self.session_timed_out = True
        return True

    def note_sample_time(self, sample: float) -> None:
        self.latest_sample_time = max(self.latest_sample_time or float("-inf"), sample)

    def encode_command(self, command: Mapping[str, Any], now: float, timeout: float) -> bytes:
        packet = dict(command)
        packet.update(self.session.command_fields(now, timeout))
        return encode_datagram(packet)
