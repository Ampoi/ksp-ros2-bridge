import json
from dataclasses import dataclass
from typing import Any, Dict, List

from .packet_conversion import sanitize_ros_name


ACTION_SELECT_CAMERA = 1
ACTION_STOP_CAMERA = 2
ACTION_RELEASE = 3
VALID_ACTIONS = {ACTION_SELECT_CAMERA, ACTION_STOP_CAMERA, ACTION_RELEASE}


@dataclass(frozen=True)
class DockingPortStateData:
    name: str
    part_flight_id: int
    module_index: int
    node_type: str
    state: str
    docked: bool
    acquiring: bool
    releasable: bool
    camera_active: bool
    partner_name: str
    partner_part_flight_id: int


def docking_port_name(value: Any) -> str:
    if not isinstance(value, str) or not value or len(value) > 128:
        raise ValueError("invalid docking port name")
    return sanitize_ros_name(value, "docking_port")


def docking_port_manifest_from_packet(packet: Dict[str, Any]) -> List[str]:
    if packet.get("type") != "ksp_docking_port_manifest" or packet.get("version") != 1:
        raise ValueError("unsupported docking port manifest")
    ports = packet.get("ports")
    if not isinstance(ports, list) or len(ports) > 256:
        raise ValueError("docking port manifest must contain a bounded ports array")
    names: List[str] = []
    seen = set()
    for entry in ports:
        if not isinstance(entry, dict):
            raise ValueError("docking port manifest entries must be objects")
        name = docking_port_name(entry.get("name"))
        if name in seen:
            raise ValueError("duplicate docking port name")
        _bounded_int(entry.get("partFlightId"), "part flight ID", 0, 2**64 - 1)
        _bounded_int(entry.get("moduleIndex"), "module index", 0, 1024)
        seen.add(name)
        names.append(name)
    return names


def docking_port_state_from_packet(packet: Dict[str, Any]) -> DockingPortStateData:
    if packet.get("type") != "ksp_docking_port_state" or packet.get("version") != 1:
        raise ValueError("unsupported docking port state")
    name = docking_port_name(packet.get("name"))
    partner_name = str(packet.get("partnerName") or "")
    if len(partner_name) > 128:
        raise ValueError("docking partner name is too long")
    if partner_name:
        partner_name = docking_port_name(partner_name)
    return DockingPortStateData(
        name=name,
        part_flight_id=_bounded_int(packet.get("partFlightId"), "part flight ID", 0, 2**64 - 1),
        module_index=_bounded_int(packet.get("moduleIndex"), "module index", 0, 1024),
        node_type=_bounded_string(packet.get("nodeType"), "node type", 128),
        state=_bounded_string(packet.get("state"), "state", 128),
        docked=_strict_bool(packet.get("docked"), "docked"),
        acquiring=_strict_bool(packet.get("acquiring"), "acquiring"),
        releasable=_strict_bool(packet.get("releasable"), "releasable"),
        camera_active=_strict_bool(packet.get("cameraActive"), "camera active"),
        partner_name=partner_name,
        partner_part_flight_id=_bounded_int(packet.get("partnerPartFlightId", 0), "partner part flight ID", 0, 2**64 - 1),
    )


def encode_docking_port_command(name: Any, action: Any, sequence: Any) -> bytes:
    normalized_name = docking_port_name(name)
    parsed_action = _bounded_int(action, "action", 1, 255)
    parsed_sequence = _bounded_int(sequence, "sequence", 1, 2**63 - 1)
    if parsed_action not in VALID_ACTIONS:
        raise ValueError("unsupported docking port action")
    return json.dumps(
        {"type": "ksp_docking_port_command", "version": 1, "name": normalized_name,
         "action": parsed_action, "sequence": parsed_sequence},
        separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")


def _strict_bool(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"docking port {label} must be boolean")
    return value


def _bounded_int(value: Any, label: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool):
        raise ValueError(f"invalid docking port {label}")
    try:
        parsed = int(value)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid docking port {label}") from exc
    if parsed < minimum or parsed > maximum:
        raise ValueError(f"docking port {label} is outside its limit")
    return parsed


def _bounded_string(value: Any, label: str, maximum: int) -> str:
    if not isinstance(value, str) or not value or len(value) > maximum:
        raise ValueError(f"invalid docking port {label}")
    return value
