import json
import math
from typing import Any, Dict, List, Mapping

from .packet_conversion import as_float, as_int, sanitize_ros_name


def _finite(value: Any, field_name: str) -> float:
    converted = as_float(value, math.nan)
    if not math.isfinite(converted):
        raise ValueError(f"{field_name} must be finite")
    return converted


def _unit_interval(value: Any, field_name: str) -> float:
    converted = _finite(value, field_name)
    if converted < 0.0 or converted > 1.0:
        raise ValueError(f"{field_name} must be between 0 and 1")
    return converted


def _axis(value: Any, field_name: str) -> float:
    converted = _finite(value, field_name)
    if converted < -1.0 or converted > 1.0:
        raise ValueError(f"{field_name} must be between -1 and 1")
    return converted


def propulsion_command_from_json(data: str, sequence: int) -> Dict[str, Any]:
    try:
        source = json.loads(data)
    except (TypeError, json.JSONDecodeError) as exc:
        raise ValueError(f"command must be valid JSON: {exc}") from exc
    if not isinstance(source, Mapping):
        raise ValueError("command must be a JSON object")

    timeout = _finite(source.get("timeout", 0.5), "timeout")
    if timeout < 0.05 or timeout > 10.0:
        raise ValueError("timeout must be between 0.05 and 10 seconds")

    raw_commands = source.get("commands", [])
    if not isinstance(raw_commands, list):
        raise ValueError("commands must be an array")

    commands: List[Dict[str, Any]] = []
    for index, item in enumerate(raw_commands):
        if not isinstance(item, Mapping):
            raise ValueError(f"commands[{index}] must be an object")
        raw_name = str(item.get("name", "")).strip()
        part_flight_id = max(0, as_int(item.get("partFlightId"), 0))
        if raw_name == "*":
            name = "*"
        elif raw_name:
            name = sanitize_ros_name(raw_name, "propulsion")
        elif part_flight_id > 0:
            name = ""
        else:
            raise ValueError(f"commands[{index}] needs name or partFlightId")

        kind = str(item.get("kind", "")).strip().lower()
        if kind not in ("", "engine", "rcs"):
            raise ValueError(f"commands[{index}].kind must be engine or rcs")

        has_enabled = "enabled" in item
        has_throttle = "throttle" in item
        release = bool(item.get("release", False))
        if not has_enabled and not has_throttle and not release:
            raise ValueError(
                f"commands[{index}] needs enabled, throttle, or release"
            )

        commands.append(
            {
                "name": name,
                "partFlightId": part_flight_id,
                "moduleIndex": max(0, as_int(item.get("moduleIndex"), 0)),
                "kind": kind,
                "hasEnabled": has_enabled,
                "enabled": bool(item.get("enabled", False)),
                "hasThrottle": has_throttle,
                "throttle": (
                    _unit_interval(item.get("throttle"), f"commands[{index}].throttle")
                    if has_throttle
                    else 0.0
                ),
                "release": release,
            }
        )

    packet: Dict[str, Any] = {
        "type": "ksp_propulsion_command",
        "version": 1,
        "commands": commands,
        "hasMainThrottle": False,
        "mainThrottle": 0.0,
        "hasRcsCommand": False,
        "rcsX": 0.0,
        "rcsY": 0.0,
        "rcsZ": 0.0,
        "rcsPitch": 0.0,
        "rcsYaw": 0.0,
        "rcsRoll": 0.0,
        "timeoutSeconds": timeout,
        "sequence": int(sequence),
    }
    if "mainThrottle" in source:
        packet["hasMainThrottle"] = True
        packet["mainThrottle"] = _unit_interval(
            source.get("mainThrottle"), "mainThrottle"
        )

    rcs = source.get("rcs")
    if rcs is not None:
        if not isinstance(rcs, Mapping):
            raise ValueError("rcs must be an object")
        packet["hasRcsCommand"] = True
        for field, wire_field in (
            ("x", "rcsX"),
            ("y", "rcsY"),
            ("z", "rcsZ"),
            ("pitch", "rcsPitch"),
            ("yaw", "rcsYaw"),
            ("roll", "rcsRoll"),
        ):
            packet[wire_field] = _axis(rcs.get(field, 0.0), f"rcs.{field}")

    if not commands and not packet["hasMainThrottle"] and not packet["hasRcsCommand"]:
        raise ValueError("command has no propulsion controls")
    return packet


def main_throttle_command(value: float, sequence: int, timeout: float = 0.5) -> Dict[str, Any]:
    return propulsion_command_from_json(
        json.dumps({"mainThrottle": value, "timeout": timeout}), sequence
    )


def rcs_command(
    x: float,
    y: float,
    z: float,
    pitch: float,
    yaw: float,
    roll: float,
    sequence: int,
    timeout: float = 0.5,
) -> Dict[str, Any]:
    return propulsion_command_from_json(
        json.dumps(
            {
                "rcs": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "pitch": pitch,
                    "yaw": yaw,
                    "roll": roll,
                },
                "timeout": timeout,
            }
        ),
        sequence,
    )


def encode_propulsion_command(command: Dict[str, Any]) -> bytes:
    return json.dumps(command, separators=(",", ":"), allow_nan=False).encode("utf-8")


def propulsion_state_json(packet: Dict[str, Any]) -> str:
    if packet.get("type") != "ksp_propulsion_state" or packet.get("version") != 1:
        raise ValueError("packet is not a supported propulsion state")
    kind = str(packet.get("kind", "")).lower()
    if kind not in ("engine", "rcs"):
        raise ValueError(f"unsupported propulsion kind: {kind}")
    name = sanitize_ros_name(packet.get("name"), kind)
    normalized = dict(packet)
    normalized["name"] = name
    normalized["kind"] = kind
    normalized["partFlightId"] = max(0, as_int(packet.get("partFlightId"), 0))
    normalized["moduleIndex"] = max(0, as_int(packet.get("moduleIndex"), 0))
    for field in ("throttleLimit", "thrust", "maxThrust"):
        normalized[field] = _finite(packet.get(field, 0.0), field)
    if kind == "engine":
        normalized["throttle"] = _finite(packet.get("throttle", 0.0), "throttle")
    return json.dumps(normalized, separators=(",", ":"), allow_nan=False, sort_keys=True)
