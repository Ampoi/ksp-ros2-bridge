from .protocol import encode_datagram
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Sequence

from .packet_conversion import as_float, as_int, sanitize_ros_name


@dataclass(frozen=True)
class MotorStateData:
    name: str
    joint_type: str
    vessel: str
    part_flight_id: int
    position: float
    velocity: float
    effort: float
    current: float
    target: float
    powered: bool
    engaged: bool
    locked: bool
    command_mode: str
    command_active: bool


def _finite_float(value: Any, field_name: str) -> float:
    converted = as_float(value, math.nan)
    if not math.isfinite(converted):
        raise ValueError(f"{field_name} must be finite")
    return converted


def motor_state_from_packet(packet: Dict[str, Any]) -> MotorStateData:
    if packet.get("type") != "pylon_motor_state":
        raise ValueError("packet is not a motor state")

    name = sanitize_ros_name(packet.get("name"), "motor")
    joint_type = str(packet.get("jointType", "")).lower()
    if joint_type not in ("revolute", "prismatic"):
        raise ValueError(f"unsupported motor joint type: {joint_type}")

    return MotorStateData(
        name=name,
        joint_type=joint_type,
        vessel=str(packet.get("vessel") or ""),
        part_flight_id=max(0, as_int(packet.get("partFlightId"), 0)),
        position=_finite_float(packet.get("position"), "position"),
        velocity=_finite_float(packet.get("velocity"), "velocity"),
        effort=_finite_float(packet.get("effort"), "effort"),
        current=max(0.0, _finite_float(packet.get("current", 0.0), "current")),
        target=_finite_float(packet.get("target"), "target"),
        powered=bool(packet.get("powered", False)),
        engaged=bool(packet.get("engaged", False)),
        locked=bool(packet.get("locked", False)),
        command_mode=str(packet.get("commandMode") or "position").lower(),
        command_active=bool(packet.get("commandActive", False)),
    )


def _optional_values(values: Sequence[float], count: int, field_name: str) -> List[float]:
    if not values:
        return []
    if len(values) != count:
        raise ValueError(f"{field_name} must be empty or match joint_names")
    return [_finite_float(value, field_name) for value in values]




def encode_motor_command(command: Dict[str, Any]) -> bytes:
    return encode_datagram(command)
