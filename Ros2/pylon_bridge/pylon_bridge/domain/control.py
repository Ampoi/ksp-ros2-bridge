"""Validated domain snapshots for the vehicle-control feedback protocol."""

from dataclasses import dataclass
import math
from typing import Any, Mapping, Tuple


Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class WrenchValue:
    force: Vector3
    torque: Vector3


@dataclass(frozen=True)
class AuthorityState:
    state: int
    vessel_id: str
    vessel_name: str
    controller_id: str
    lease_id: str
    priority: int
    lease_remaining: float
    sas_suppressed: bool
    emergency_stop: bool
    last_sequence: int
    reason: str


@dataclass(frozen=True)
class WrenchFeedbackValue:
    vessel_id: str
    controller_id: str
    lease_id: str
    sequence: int
    accepted: bool
    reason: str
    requested: WrenchValue
    allocated: WrenchValue
    achieved: WrenchValue
    allocation_residual: WrenchValue
    tracking_residual: WrenchValue
    saturation_ratio: float
    tracking_error_ratio: float
    saturated: bool
    achieved_quality: str


def authority_state_from_packet(packet: Mapping[str, Any]) -> AuthorityState:
    if packet.get("type") != "pylon_control_authority_state" or packet.get("version") != 1:
        raise ValueError("unsupported control authority state")
    state = _integer(packet.get("state"), "state")
    if state not in (0, 1, 2):
        raise ValueError("authority state is out of range")
    return AuthorityState(
        state=state,
        vessel_id=_string(packet.get("vesselId"), "vesselId"),
        vessel_name=_string(packet.get("vessel", ""), "vessel"),
        controller_id=_string(packet.get("controllerId", ""), "controllerId"),
        lease_id=_string(packet.get("leaseId", ""), "leaseId"),
        priority=_integer(packet.get("priority", 0), "priority"),
        lease_remaining=max(0.0, _finite(packet.get("leaseRemainingSeconds", 0.0), "leaseRemainingSeconds")),
        sas_suppressed=bool(packet.get("sasSuppressed", False)),
        emergency_stop=bool(packet.get("emergencyStop", False)),
        last_sequence=max(0, _integer(packet.get("lastSequence", 0), "lastSequence")),
        reason=_string(packet.get("reason", ""), "reason"),
    )


def wrench_feedback_from_packet(packet: Mapping[str, Any]) -> WrenchFeedbackValue:
    if packet.get("type") != "pylon_wrench_status" or packet.get("version") != 1:
        raise ValueError("unsupported wrench feedback")
    return WrenchFeedbackValue(
        vessel_id=_string(packet.get("vesselId"), "vesselId"),
        controller_id=_string(packet.get("controllerId", ""), "controllerId"),
        lease_id=_string(packet.get("leaseId", ""), "leaseId"),
        sequence=max(0, _integer(packet.get("sequence", 0), "sequence")),
        accepted=bool(packet.get("accepted", False)),
        reason=_string(packet.get("reason", ""), "reason"),
        requested=_wrench(packet.get("requested"), "requested"),
        allocated=_wrench(packet.get("allocated"), "allocated"),
        achieved=_wrench(packet.get("achieved"), "achieved"),
        allocation_residual=_wrench(packet.get("allocationResidual"), "allocationResidual"),
        tracking_residual=_wrench(packet.get("trackingResidual"), "trackingResidual"),
        saturation_ratio=max(0.0, _finite(packet.get("saturationRatio"), "saturationRatio")),
        tracking_error_ratio=max(0.0, _finite(packet.get("trackingErrorRatio"), "trackingErrorRatio")),
        saturated=bool(packet.get("saturated", False)),
        achieved_quality=_string(packet.get("achievedQuality", ""), "achievedQuality"),
    )


def _wrench(value: Any, label: str) -> WrenchValue:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be an object")
    return WrenchValue(_vector(value.get("force"), f"{label}.force"), _vector(value.get("torque"), f"{label}.torque"))


def _vector(value: Any, label: str) -> Vector3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{label} must contain three values")
    return tuple(_finite(component, label) for component in value)  # type: ignore[return-value]


def _finite(value: Any, label: str) -> float:
    try:
        converted = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must be numeric") from exc
    if not math.isfinite(converted):
        raise ValueError(f"{label} must be finite")
    return converted


def _integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} must be an integer")
    return value


def _string(value: Any, label: str) -> str:
    if not isinstance(value, str) or len(value) > 128:
        raise ValueError(f"{label} must be a bounded string")
    return value
