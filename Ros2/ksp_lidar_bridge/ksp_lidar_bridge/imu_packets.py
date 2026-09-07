"""The default vessel IMU wire contract (SI, body forward/left/up)."""
from dataclasses import dataclass
import re
from typing import Any, Mapping

from .vehicle_packets import Vector3, _finite, _vector


@dataclass(frozen=True)
class ImuData:
    vessel_id: str
    universal_time: float
    angular_velocity: Vector3
    linear_acceleration: Vector3


def imu_from_packet(packet: Mapping[str, Any]) -> ImuData:
    if packet.get("type") != "ksp_imu" or packet.get("version") != 1:
        raise ValueError("packet is not a supported vessel IMU")
    vessel_id = str(packet.get("vesselId") or "")
    if re.fullmatch(r"[0-9a-f]{32}", vessel_id) is None:
        raise ValueError("vesselId must be a lowercase KSP UUID without hyphens")
    return ImuData(
        vessel_id,
        _finite(packet.get("universalTime"), "universalTime"),
        _vector(packet, "angularVelocity", 3),
        _vector(packet, "linearAcceleration", 3),
    )
