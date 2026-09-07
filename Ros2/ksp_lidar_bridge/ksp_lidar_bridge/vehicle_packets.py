import json
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Set, Tuple

from .packet_conversion import as_float, as_int, sanitize_ros_name


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
SUPPORTED_ACTUATOR_KINDS = ("wheel", "engine", "rcs", "motor", "separation")
SUPPORTED_SEPARATION_MECHANISMS = ("decoupler", "fairing")


def _finite(value: Any, field: str) -> float:
    converted = as_float(value, math.nan)
    if not math.isfinite(converted):
        raise ValueError(f"{field} must be finite")
    return converted


def _vector(packet: Mapping[str, Any], field: str, count: int) -> Tuple[float, ...]:
    raw = packet.get(field)
    if not isinstance(raw, (list, tuple)) or len(raw) != count:
        raise ValueError(f"{field} must contain {count} values")
    return tuple(_finite(value, field) for value in raw)


def _optional_vector(
    packet: Mapping[str, Any], field: str, count: int
) -> Optional[Tuple[float, ...]]:
    if field not in packet:
        return None
    return _vector(packet, field, count)


@dataclass(frozen=True)
class GroundTruthData:
    vessel_id: str
    vessel_name: str
    origin_sequence: int
    universal_time: float
    position: Vector3
    rotation: Quaternion
    linear_velocity: Vector3
    angular_velocity: Vector3
    linear_velocity_body: Optional[Vector3]
    angular_velocity_body: Optional[Vector3]
    linear_acceleration: Vector3
    angular_acceleration: Vector3
    frame_angular_velocity: Optional[Vector3] = None


@dataclass(frozen=True)
class NearbyVesselData:
    vessel_id: str
    vessel_name: str
    is_debris: bool
    position: Vector3
    linear_velocity: Vector3


@dataclass(frozen=True)
class NearbyVesselsData:
    observer_vessel_id: str
    origin_sequence: int
    universal_time: float
    observer_position: Vector3
    observer_linear_velocity: Vector3
    vessels: Tuple[NearbyVesselData, ...]


def nearby_vessels_from_packet(packet: Mapping[str, Any]) -> NearbyVesselsData:
    if packet.get("type") != "ksp_nearby_vessels" or packet.get("version") != 1:
        raise ValueError("packet is not supported nearby vessel truth")
    observer = str(packet.get("vesselId") or "")
    raw = packet.get("vessels")
    if not observer or not isinstance(raw, list) or len(raw) > 32:
        raise ValueError("nearby truth requires an observer and at most 32 vessels")
    vessels = []
    seen = {observer}
    for item in raw:
        if not isinstance(item, Mapping):
            raise ValueError("nearby vessel must be an object")
        vessel_id = str(item.get("vesselId") or "")
        if not vessel_id or vessel_id in seen or not isinstance(item.get("isDebris"), bool):
            raise ValueError("nearby vessel IDs must be unique and isDebris boolean")
        seen.add(vessel_id)
        vessels.append(NearbyVesselData(
            vessel_id, str(item.get("vessel") or ""), item["isDebris"],
            _vector(item, "position", 3), _vector(item, "linearVelocity", 3),
        ))
    return NearbyVesselsData(
        observer, max(0, as_int(packet.get("originSequence"), 0)),
        _finite(packet.get("universalTime"), "universalTime"),
        _vector(packet, "position", 3), _vector(packet, "linearVelocity", 3),
        tuple(vessels),
    )


def ground_truth_from_packet(packet: Mapping[str, Any]) -> GroundTruthData:
    if packet.get("type") != "ksp_ground_truth" or packet.get("version") != 1:
        raise ValueError("packet is not supported ground truth")
    rotation = _vector(packet, "rotation", 4)
    magnitude = math.sqrt(sum(value * value for value in rotation))
    if magnitude < 1e-9:
        raise ValueError("rotation must be a non-zero quaternion")
    rotation = tuple(value / magnitude for value in rotation)
    return GroundTruthData(
        vessel_id=str(packet.get("vesselId") or ""),
        vessel_name=str(packet.get("vessel") or ""),
        origin_sequence=max(0, as_int(packet.get("originSequence"), 0)),
        universal_time=_finite(packet.get("universalTime", 0.0), "universalTime"),
        position=_vector(packet, "position", 3),  # type: ignore[arg-type]
        rotation=rotation,  # type: ignore[arg-type]
        linear_velocity=_vector(packet, "linearVelocity", 3),  # type: ignore[arg-type]
        angular_velocity=_vector(packet, "angularVelocity", 3),  # type: ignore[arg-type]
        linear_velocity_body=_optional_vector(  # type: ignore[arg-type]
            packet, "linearVelocityBody", 3
        ),
        angular_velocity_body=_optional_vector(  # type: ignore[arg-type]
            packet, "angularVelocityBody", 3
        ),
        linear_acceleration=_vector(packet, "linearAcceleration", 3),  # type: ignore[arg-type]
        angular_acceleration=_vector(packet, "angularAcceleration", 3),  # type: ignore[arg-type]
        frame_angular_velocity=_optional_vector(packet, "frameAngularVelocity", 3),
    )


def actuator_name(packet: Mapping[str, Any], fallback: str = "actuator") -> str:
    return sanitize_ros_name(packet.get("name"), fallback)


def actuator_state_from_packet(packet: Mapping[str, Any]) -> Dict[str, Any]:
    if packet.get("type") != "ksp_actuator_state" or packet.get("version") != 1:
        raise ValueError("packet is not a supported actuator state")
    kind = str(packet.get("actuatorType") or "").lower()
    if kind not in SUPPORTED_ACTUATOR_KINDS:
        raise ValueError(f"unsupported actuator type: {kind}")
    state = dict(packet)
    state["actuatorType"] = kind
    state["name"] = actuator_name(packet, kind)
    finite_fields = {
        "wheel": (
            "angularPosition", "angularVelocity", "steeringAngle", "driveTorque",
            "brakeTorque", "slip", "maxDriveTorque",
        ),
        "engine": ("throttle", "thrust", "maxThrust"),
        "rcs": ("thrust", "maxThrust", "thrustLimit"),
        "motor": ("position", "velocity", "effort", "target", "current"),
        "separation": (),
    }[kind]
    for field in finite_fields:
        state[field] = _finite(packet.get(field, 0.0), field)
    if kind == "separation":
        mechanism = str(packet.get("mechanism") or "").lower()
        if mechanism not in SUPPORTED_SEPARATION_MECHANISMS:
            raise ValueError(f"unsupported separation mechanism: {mechanism}")
        state["mechanism"] = mechanism
    return state


def actuator_manifest_from_packet(packet: Mapping[str, Any]) -> Dict[str, str]:
    if packet.get("type") != "ksp_actuator_manifest" or packet.get("version") != 1:
        raise ValueError("packet is not a supported actuator manifest")
    raw_actuators = packet.get("actuators")
    if not isinstance(raw_actuators, list):
        raise ValueError("actuators must be a list")
    manifest: Dict[str, str] = {}
    for raw in raw_actuators:
        if not isinstance(raw, Mapping):
            raise ValueError("each actuator must be an object")
        kind = str(raw.get("actuatorType") or "").lower()
        if kind not in SUPPORTED_ACTUATOR_KINDS:
            raise ValueError(f"unsupported actuator type: {kind}")
        name = actuator_name(raw, kind)
        if name in manifest:
            raise ValueError(f"duplicate actuator name: {name}")
        manifest[name] = kind
    return manifest


def actuator_names_to_remove(
    manifest: Mapping[str, str],
    current_kinds: Mapping[str, str],
    latched_separations: Set[str],
    vessel_changed: bool,
) -> List[str]:
    if vessel_changed:
        return list(current_kinds)
    return [
        name
        for name, kind in current_kinds.items()
        if name not in manifest
        and not (kind == "separation" and name in latched_separations)
    ]


def body_wrench_command(
    force: Vector3,
    torque: Vector3,
    sequence: int,
    timeout: float,
    vessel_id: str,
    controller_id: str,
    lease_id: str,
) -> Dict[str, Any]:
    values = tuple(force) + tuple(torque) + (timeout,)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("body wrench values must be finite")
    if timeout < 0.05 or timeout > 10.0:
        raise ValueError("timeout must be between 0.05 and 10 seconds")
    return {
        "type": "ksp_body_wrench_command",
        "version": 2,
        "frame": "base_link",
        "vesselId": _bounded_identity(vessel_id, "vessel_id"),
        "controllerId": _bounded_identity(controller_id, "controller_id"),
        "leaseId": _bounded_identity(lease_id, "lease_id"),
        "force": list(force),
        "torque": list(torque),
        "timeoutSeconds": float(timeout),
        "sequence": _positive_sequence(sequence),
    }


def control_authority_command(
    action: str,
    vessel_id: str,
    controller_id: str,
    lease_id: str,
    priority: int,
    lease_duration: float,
    suppress_sas: bool,
    sequence: int,
) -> Dict[str, Any]:
    normalized_action = str(action or "").strip().lower()
    if normalized_action not in {
        "acquire", "renew", "release", "emergency_stop", "clear_emergency_stop"
    }:
        raise ValueError("unsupported control authority action")
    duration = _finite(lease_duration, "lease_duration")
    if normalized_action in {"acquire", "renew"} and not 0.1 <= duration <= 10.0:
        raise ValueError("lease_duration must be between 0.1 and 10 seconds")
    if isinstance(priority, bool) or not isinstance(priority, int):
        raise ValueError("priority must be an integer")
    return {
        "type": "ksp_control_authority_command",
        "version": 2,
        "action": normalized_action,
        "vesselId": _bounded_identity(vessel_id, "vessel_id"),
        "controllerId": _bounded_identity(controller_id, "controller_id"),
        "leaseId": _bounded_identity(lease_id, "lease_id"),
        "priority": priority,
        "leaseDurationSeconds": duration,
        "suppressSas": bool(suppress_sas),
        "sequence": _positive_sequence(sequence),
    }


def _bounded_identity(value: Any, field: str) -> str:
    normalized = str(value or "").strip()
    if not normalized or len(normalized) > 64:
        raise ValueError(f"{field} must contain 1 to 64 characters")
    return normalized


def _positive_sequence(value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError("sequence must be a positive integer")
    return value


def actuator_command(
    kind: str,
    name: str,
    values: Mapping[str, Any],
    sequence: int,
    vessel_id: str,
    controller_id: str,
    lease_id: str,
) -> Dict[str, Any]:
    normalized_kind = str(kind or "").lower()
    if normalized_kind not in SUPPORTED_ACTUATOR_KINDS:
        raise ValueError(f"unsupported actuator type: {normalized_kind}")
    normalized_name = sanitize_ros_name(name, normalized_kind)
    timeout = _finite(values.get("timeoutSeconds", 0.5), "timeoutSeconds")
    if timeout < 0.05 or timeout > 10.0:
        raise ValueError("timeoutSeconds must be between 0.05 and 10 seconds")
    command: Dict[str, Any] = {
        "type": "ksp_actuator_command",
        "version": 2,
        "actuatorType": normalized_kind,
        "name": normalized_name,
        "vesselId": _bounded_identity(vessel_id, "vessel_id"),
        "controllerId": _bounded_identity(controller_id, "controller_id"),
        "leaseId": _bounded_identity(lease_id, "lease_id"),
        "enabled": bool(values.get("enabled", True)),
        "timeoutSeconds": timeout,
        "sequence": _positive_sequence(sequence),
    }
    for key, value in values.items():
        if key in ("enabled", "timeoutSeconds"):
            continue
        if isinstance(value, bool):
            command[key] = value
        else:
            command[key] = _finite(value, key) if isinstance(value, (int, float)) else value
    return command


def encode_vehicle_command(command: Mapping[str, Any]) -> bytes:
    return json.dumps(command, separators=(",", ":"), allow_nan=False).encode("utf-8")
