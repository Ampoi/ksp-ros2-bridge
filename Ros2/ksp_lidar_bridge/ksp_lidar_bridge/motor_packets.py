import json
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


def _finite_float(value: Any, field_name: str) -> float:
    converted = as_float(value, math.nan)
    if not math.isfinite(converted):
        raise ValueError(f"{field_name} must be finite")
    return converted


def motor_state_from_packet(packet: Dict[str, Any]) -> MotorStateData:
    if packet.get("type") != "ksp_motor_state":
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
    )


def _optional_values(values: Sequence[float], count: int, field_name: str) -> List[float]:
    if not values:
        return []
    if len(values) != count:
        raise ValueError(f"{field_name} must be empty or match joint_names")
    return [_finite_float(value, field_name) for value in values]


def motor_commands_from_point(
    joint_names: Sequence[str],
    positions: Sequence[float],
    velocities: Sequence[float],
    efforts: Sequence[float],
    sequence: int,
) -> List[Dict[str, Any]]:
    if not joint_names:
        raise ValueError("joint_names must not be empty")

    names = [sanitize_ros_name(name, "motor") for name in joint_names]
    if len(set(names)) != len(names):
        raise ValueError("joint_names must be unique after ROS name sanitization")

    count = len(names)
    point_positions = _optional_values(positions, count, "positions")
    point_velocities = _optional_values(velocities, count, "velocities")
    point_efforts = _optional_values(efforts, count, "effort")
    if not point_positions and not point_velocities and not point_efforts:
        raise ValueError("trajectory point has no position, velocity, or effort values")

    mode = "position" if point_positions else "velocity" if point_velocities else "effort"
    commands: List[Dict[str, Any]] = []
    for index, name in enumerate(names):
        commands.append(
            {
                "type": "ksp_motor_command",
                "version": 1,
                "name": name,
                "partFlightId": 0,
                "mode": mode,
                "hasPosition": bool(point_positions),
                "position": point_positions[index] if point_positions else 0.0,
                "hasVelocity": bool(point_velocities),
                "velocity": point_velocities[index] if point_velocities else 0.0,
                "hasEffort": bool(point_efforts),
                "effort": point_efforts[index] if point_efforts else 0.0,
                "sequence": int(sequence),
            }
        )
    return commands


def encode_motor_command(command: Dict[str, Any]) -> bytes:
    return json.dumps(command, separators=(",", ":"), allow_nan=False).encode("utf-8")
