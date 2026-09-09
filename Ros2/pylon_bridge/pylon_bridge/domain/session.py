"""Transport-independent flight sessions; only identity heartbeats can open a session."""

from dataclasses import dataclass
import math
import uuid

from ..protocol import SESSION_PACKET_TYPE


@dataclass(frozen=True)
class SessionKey:
    instance: str
    generation: int
    epoch: str
    vessel: str

    @classmethod
    def from_packet(cls, packet):
        values = [packet.get(key) for key in (
            "runtimeInstance", "runtimeEpoch", "runtimeVesselId")]
        generation = packet.get("runtimeGeneration")
        if any(not isinstance(value, str) for value in values) or not values[0] or not values[1]:
            raise ValueError("missing runtime session identity")
        if type(generation) is not int or generation < 0:
            raise ValueError("invalid runtime generation")
        return cls(values[0], generation, values[1], values[2])

    def fields(self):
        return dict(runtimeInstance=self.instance, runtimeGeneration=self.generation,
                    runtimeEpoch=self.epoch, runtimeVesselId=self.vessel)


class SessionTracker:
    def __init__(self):
        self.key = None
        self.retired_instances = set()
        self.last_seen = 0.0
        self.available = False
        self.vessel_name = ""
        self.sample_time = None
        self.revision = 0
        self.generation = uuid.uuid4().int & ((1 << 63) - 1)

    def observe(self, packet, now):
        candidate = SessionKey.from_packet(packet)
        if packet.get("type") != SESSION_PACKET_TYPE:
            return False
        if type(packet.get("available")) is not bool:
            raise ValueError("session availability must be boolean")
        stamp = packet.get("universalTime")
        if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
                or not math.isfinite(stamp)):
            raise ValueError("invalid session sample time")
        if packet.get("vesselId") != candidate.vessel:
            raise ValueError("session vessel identity mismatch")
        if candidate.instance in self.retired_instances:
            return False
        if self.key:
            if candidate.instance == self.key.instance:
                if candidate.generation < self.key.generation:
                    return False
                if candidate.generation == self.key.generation:
                    if candidate != self.key:
                        return False
                    if self.sample_time is not None and stamp < self.sample_time:
                        return False
            else:
                self.retired_instances.add(self.key.instance)
        changed = candidate != self.key or packet["available"] != self.available
        if changed:
            self.revision += 1
            self.generation += 1
        self.key = candidate
        self.last_seen = now
        self.available = packet["available"]
        self.vessel_name = str(packet.get("vesselName", ""))
        self.sample_time = stamp
        return True

    def accepts(self, packet):
        if not self.available or self.key is None:
            return False
        try:
            key = SessionKey.from_packet(packet)
        except ValueError:
            return False
        return key == self.key and packet.get("vesselId", key.vessel) == key.vessel

    def command_fields(self, now, timeout):
        if self.key is None or not self.available or now - self.last_seen >= timeout:
            raise ValueError("no fresh active flight session")
        return self.key.fields()
