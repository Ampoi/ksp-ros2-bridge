"""Model transfer admission and invalidation, independent of ROS publication."""

import ipaddress

from ..vessel_model import UrdfChunkAssembler


class ModelTransfer:
    def __init__(self):
        self.assembler = UrdfChunkAssembler()
        self.cleared_sessions = {}

    def consume(self, packet, now):
        self._expire_cleared(now)
        if packet.get("sessionId") in self.cleared_sessions:
            return None
        return self.assembler.consume(packet, now=now)

    def clear(self, packet, now):
        session_id = packet.get("sessionId")
        if (packet.get("version") != 1 or not isinstance(session_id, str)
                or len(session_id) != 32
                or any(character not in "0123456789abcdef" for character in session_id)):
            return None
        self._expire_cleared(now)
        if len(self.cleared_sessions) >= 1024:
            oldest = min(self.cleared_sessions, key=self.cleared_sessions.get)
            del self.cleared_sessions[oldest]
        self.cleared_sessions[session_id] = now + 120.0
        return session_id

    def _expire_cleared(self, now):
        self.cleared_sessions = {
            session: expires_at for session, expires_at in self.cleared_sessions.items()
            if expires_at > now
        }

    @staticmethod
    def source_allowed(address, allow_remote=False):
        if allow_remote:
            return True
        try:
            source = ipaddress.ip_address(address[0])
            return source.is_loopback or (
                source.version == 6 and source.ipv4_mapped is not None
                and source.ipv4_mapped.is_loopback
            )
        except (IndexError, TypeError, ValueError):
            return False
