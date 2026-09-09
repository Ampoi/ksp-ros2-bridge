"""PyLoN v1 datagram framing, independent of sockets and ROS messages."""

import json
from typing import Any, Dict, Mapping

PROTOCOL_VERSION = 1
SESSION_PACKET_TYPE = "pylon_session"


def decode_datagram(data: bytes) -> Dict[str, Any]:
    packet = json.loads(data.decode("utf-8"))
    if not isinstance(packet, dict):
        raise ValueError("packet root must be a JSON object")
    packet_type = packet.get("type")
    if not isinstance(packet_type, str) or not packet_type.startswith("pylon_"):
        raise ValueError("unsupported protocol; expected PyLoN")
    if type(packet.get("version")) is not int or packet["version"] != PROTOCOL_VERSION:
        raise ValueError("unsupported PyLoN protocol version")
    return packet


def encode_datagram(packet: Mapping[str, Any]) -> bytes:
    return json.dumps(packet, separators=(",", ":"), allow_nan=False).encode("utf-8")
