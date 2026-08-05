import base64
import binascii
import gzip
import hashlib
import io
import json
import math
import re
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
_SAFE_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]{0,127}$")
_SESSION_ID = re.compile(r"^[0-9a-f]{32}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_ALLOWED_TAG_ATTRIBUTES = {
    "robot": {"name"},
    "link": {"name"},
    "joint": {"name", "type"},
    "parent": {"link"},
    "child": {"link"},
    "origin": {"xyz", "rpy"},
    "inertial": set(),
    "mass": {"value"},
    "inertia": {"ixx", "ixy", "ixz", "iyy", "iyz", "izz"},
    "visual": set(),
    "collision": set(),
    "geometry": set(),
    "box": {"size"},
}


@dataclass(frozen=True)
class JointTransform:
    parent_frame: str
    child_frame: str
    translation: Vector3
    rotation: Quaternion


@dataclass(frozen=True)
class VesselProxyModel:
    session_id: str
    model_id: str
    urdf: str
    root_frame: str
    part_frames: Dict[int, str]
    transforms: List[JointTransform]
    expires_after_sec: float


@dataclass
class _ChunkAssembly:
    chunk_count: int
    sha256: str
    expires_after_sec: float
    chunks: Dict[int, bytes]
    updated_at: float


class UrdfChunkAssembler:
    """Reassembles bounded runtime proxy packets without writing them to disk."""

    def __init__(
        self,
        max_chunks: int = 512,
        max_compressed_bytes: int = 4 * 1024 * 1024,
        max_uncompressed_bytes: int = 8 * 1024 * 1024,
        assembly_timeout_sec: float = 10.0,
    ) -> None:
        self.max_chunks = max_chunks
        self.max_compressed_bytes = max_compressed_bytes
        self.max_uncompressed_bytes = max_uncompressed_bytes
        self.assembly_timeout_sec = assembly_timeout_sec
        self._assemblies: Dict[Tuple[str, str], _ChunkAssembly] = {}

    def consume(
        self, packet: Dict[str, Any], received_at: Optional[float] = None
    ) -> Optional[VesselProxyModel]:
        now = time.monotonic() if received_at is None else received_at
        self._purge_stale(now)
        if packet.get("type") != "ksp_vessel_urdf_chunk":
            raise ValueError("not an active vessel URDF chunk")
        if _strict_int(packet.get("version"), "version") != 1:
            raise ValueError("unsupported active vessel URDF version")
        if packet.get("encoding") != "gzip+base64":
            raise ValueError("unsupported active vessel URDF encoding")

        session_id = _strict_string(packet.get("sessionId"), "sessionId")
        model_id = _strict_string(packet.get("modelId"), "modelId")
        digest = _strict_string(packet.get("sha256"), "sha256")
        if not _SESSION_ID.fullmatch(session_id):
            raise ValueError("invalid sessionId")
        if not _SHA256.fullmatch(model_id) or not _SHA256.fullmatch(digest):
            raise ValueError("invalid SHA-256 identifier")

        chunk_count = _strict_int(packet.get("chunkCount"), "chunkCount")
        chunk_index = _strict_int(packet.get("chunkIndex"), "chunkIndex")
        if chunk_count < 1 or chunk_count > self.max_chunks:
            raise ValueError("chunkCount is out of bounds")
        if chunk_index < 0 or chunk_index >= chunk_count:
            raise ValueError("chunkIndex is out of bounds")

        expires_after_sec = _strict_float(packet.get("expiresAfterSec"), "expiresAfterSec")
        if expires_after_sec < 1.0 or expires_after_sec > 120.0:
            raise ValueError("expiresAfterSec is out of bounds")

        encoded = _strict_string(packet.get("data"), "data")
        if len(encoded) > self.max_compressed_bytes * 2:
            raise ValueError("encoded URDF chunk is too large")
        try:
            chunk = base64.b64decode(encoded, validate=True)
        except (binascii.Error, ValueError) as exc:
            raise ValueError("invalid base64 URDF chunk") from exc
        if len(chunk) > self.max_compressed_bytes:
            raise ValueError("decoded URDF chunk is too large")

        key = (session_id, model_id)
        assembly = self._assemblies.get(key)
        if assembly is None:
            assembly = _ChunkAssembly(
                chunk_count=chunk_count,
                sha256=digest,
                expires_after_sec=expires_after_sec,
                chunks={},
                updated_at=now,
            )
            self._assemblies[key] = assembly
        elif assembly.chunk_count != chunk_count or assembly.sha256 != digest:
            del self._assemblies[key]
            raise ValueError("inconsistent URDF chunk metadata")

        previous = assembly.chunks.get(chunk_index)
        if previous is not None and previous != chunk:
            del self._assemblies[key]
            raise ValueError("conflicting duplicate URDF chunk")
        assembly.chunks[chunk_index] = chunk
        assembly.updated_at = now

        compressed_size = sum(len(value) for value in assembly.chunks.values())
        if compressed_size > self.max_compressed_bytes:
            del self._assemblies[key]
            raise ValueError("compressed active vessel URDF exceeds size limit")
        if len(assembly.chunks) != assembly.chunk_count:
            return None

        del self._assemblies[key]
        compressed = b"".join(assembly.chunks[index] for index in range(chunk_count))
        if hashlib.sha256(compressed).hexdigest() != assembly.sha256:
            raise ValueError("active vessel URDF checksum mismatch")
        bundle_bytes = _bounded_gzip_decompress(compressed, self.max_uncompressed_bytes)
        try:
            bundle = json.loads(bundle_bytes.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("invalid active vessel URDF bundle") from exc
        return _model_from_bundle(
            bundle,
            expected_session_id=session_id,
            expected_model_id=model_id,
            expires_after_sec=assembly.expires_after_sec,
            max_urdf_bytes=self.max_uncompressed_bytes,
        )

    def _purge_stale(self, now: float) -> None:
        stale = [
            key
            for key, assembly in self._assemblies.items()
            if now - assembly.updated_at > self.assembly_timeout_sec
        ]
        for key in stale:
            del self._assemblies[key]


def _bounded_gzip_decompress(data: bytes, maximum: int) -> bytes:
    try:
        with gzip.GzipFile(fileobj=io.BytesIO(data), mode="rb") as stream:
            output = stream.read(maximum + 1)
    except (EOFError, OSError) as exc:
        raise ValueError("invalid gzip active vessel URDF bundle") from exc
    if len(output) > maximum:
        raise ValueError("expanded active vessel URDF exceeds size limit")
    return output


def _model_from_bundle(
    bundle: Any,
    expected_session_id: str,
    expected_model_id: str,
    expires_after_sec: float,
    max_urdf_bytes: int,
) -> VesselProxyModel:
    if not isinstance(bundle, dict):
        raise ValueError("active vessel URDF bundle must be an object")
    if bundle.get("type") != "ksp_active_vessel_proxy" or bundle.get("version") != 1:
        raise ValueError("unsupported active vessel URDF bundle")
    if bundle.get("sessionId") != expected_session_id:
        raise ValueError("active vessel URDF session mismatch")
    if bundle.get("modelId") != expected_model_id:
        raise ValueError("active vessel URDF model mismatch")
    if bundle.get("geometryPolicy") != "primitive_proxy_only":
        raise ValueError("active vessel URDF must use primitive proxy geometry")
    if bundle.get("persistencePolicy") != "memory_only":
        raise ValueError("active vessel URDF must be marked memory-only")

    urdf = _strict_string(bundle.get("urdf"), "urdf")
    if len(urdf.encode("utf-8")) > max_urdf_bytes:
        raise ValueError("active vessel URDF exceeds size limit")
    if hashlib.sha256(urdf.encode("utf-8")).hexdigest() != expected_model_id:
        raise ValueError("active vessel URDF model hash mismatch")

    name_prefix = "ksp_" + expected_session_id[:8]
    root_frame, link_names, transforms = _parse_proxy_urdf(urdf, name_prefix)
    if bundle.get("rootFrame") != root_frame:
        raise ValueError("active vessel URDF root frame mismatch")

    mappings = bundle.get("partFrames")
    if not isinstance(mappings, list) or len(mappings) != len(link_names):
        raise ValueError("partFrames must map every proxy link")
    part_frames: Dict[int, str] = {}
    mapped_links = set()
    for mapping in mappings:
        if not isinstance(mapping, dict):
            raise ValueError("partFrames entries must be objects")
        part_id = _strict_int(mapping.get("partFlightId"), "partFlightId")
        frame = _strict_string(mapping.get("frame"), "frame")
        if part_id < 0 or part_id > 0xFFFFFFFFFFFFFFFF:
            raise ValueError("partFlightId is out of bounds")
        if frame not in link_names:
            raise ValueError("partFrames references an unknown link")
        if part_id in part_frames or frame in mapped_links:
            raise ValueError("partFrames contains duplicate mappings")
        part_frames[part_id] = frame
        mapped_links.add(frame)

    return VesselProxyModel(
        session_id=expected_session_id,
        model_id=expected_model_id,
        urdf=urdf,
        root_frame=root_frame,
        part_frames=part_frames,
        transforms=transforms,
        expires_after_sec=expires_after_sec,
    )


def _parse_proxy_urdf(
    urdf: str, expected_name_prefix: str
) -> Tuple[str, set, List[JointTransform]]:
    upper = urdf.upper()
    if "<!DOCTYPE" in upper or "<!ENTITY" in upper:
        raise ValueError("DTD and entity declarations are not allowed in proxy URDF")
    try:
        root = ET.fromstring(urdf)
    except ET.ParseError as exc:
        raise ValueError("invalid proxy URDF XML") from exc
    elements = list(root.iter())
    if len(elements) > 20000:
        raise ValueError("proxy URDF contains too many XML elements")
    if root.tag != "robot" or set(root.attrib) != {"name"}:
        raise ValueError("proxy URDF root must be a named robot")
    robot_name = _validate_name(root.attrib["name"], "robot name")
    if robot_name != expected_name_prefix + "_active_vessel":
        raise ValueError("proxy robot name does not match its session")
    if any(child.tag not in {"link", "joint"} for child in root):
        raise ValueError("proxy robot may only contain links and joints")

    for element in elements:
        allowed_attributes = _ALLOWED_TAG_ATTRIBUTES.get(element.tag)
        if allowed_attributes is None:
            raise ValueError(f"URDF element is not allowed: {element.tag}")
        if not set(element.attrib).issubset(allowed_attributes):
            raise ValueError(f"URDF attributes are not allowed on: {element.tag}")
        if element.text and element.text.strip():
            raise ValueError(f"URDF text content is not allowed in: {element.tag}")
        if element.tail and element.tail.strip():
            raise ValueError(f"URDF trailing text is not allowed after: {element.tag}")
        if element.tag == "origin":
            _parse_vector(element.attrib.get("xyz", "0 0 0"), "origin xyz")
            _parse_vector(element.attrib.get("rpy", "0 0 0"), "origin rpy")
        elif element.tag == "box":
            size = _parse_vector(element.attrib.get("size"), "box size")
            if any(value <= 0.0 for value in size):
                raise ValueError("proxy box dimensions must be positive")
        elif element.tag == "mass":
            if _strict_float(element.attrib.get("value"), "mass") <= 0.0:
                raise ValueError("proxy mass must be positive")
        elif element.tag == "inertia":
            required = {"ixx", "ixy", "ixz", "iyy", "iyz", "izz"}
            if set(element.attrib) != required:
                raise ValueError("proxy inertia must contain all six components")
            for name in required:
                _strict_float(element.attrib[name], f"inertia {name}")

    links: Dict[str, ET.Element] = {}
    for link in root.findall("link"):
        if set(link.attrib) != {"name"}:
            raise ValueError("proxy links must have exactly one name")
        name = _validate_name(link.attrib["name"], "link name")
        if not name.startswith(expected_name_prefix + "_link_"):
            raise ValueError("proxy link name does not match its session")
        if name in links:
            raise ValueError("proxy URDF contains duplicate links")
        if any(child.tag not in {"inertial", "visual", "collision"} for child in link):
            raise ValueError("proxy link contains an invalid child element")
        if [child.tag for child in link].count("inertial") != 1:
            raise ValueError("proxy link must contain exactly one inertial element")
        if [child.tag for child in link].count("visual") != 1:
            raise ValueError("proxy link must contain exactly one visual element")
        if [child.tag for child in link].count("collision") != 1:
            raise ValueError("proxy link must contain exactly one collision element")
        for geometry_owner in link.findall("visual") + link.findall("collision"):
            geometries = geometry_owner.findall("geometry")
            if len(geometries) != 1 or len(geometries[0].findall("box")) != 1:
                raise ValueError("proxy visual and collision must contain one box")
        links[name] = link
    if not links or len(links) > 2048:
        raise ValueError("proxy URDF link count is out of bounds")

    transforms: List[JointTransform] = []
    child_to_parent: Dict[str, str] = {}
    joint_names = set()
    for joint in root.findall("joint"):
        if set(joint.attrib) != {"name", "type"} or joint.attrib["type"] != "fixed":
            raise ValueError("proxy URDF only permits named fixed joints")
        joint_name = _validate_name(joint.attrib["name"], "joint name")
        if not joint_name.startswith(expected_name_prefix + "_joint_"):
            raise ValueError("proxy joint name does not match its session")
        if joint_name in joint_names:
            raise ValueError("proxy URDF contains duplicate joints")
        joint_names.add(joint_name)
        if any(child.tag not in {"parent", "child", "origin"} for child in joint):
            raise ValueError("proxy joint contains an invalid child element")
        if len(joint.findall("parent")) != 1 or len(joint.findall("child")) != 1:
            raise ValueError("proxy joint must contain one parent and one child")
        if len(joint.findall("origin")) != 1:
            raise ValueError("proxy joint must contain exactly one origin")
        parent_element = joint.find("parent")
        child_element = joint.find("child")
        if parent_element is None or child_element is None:
            raise ValueError("proxy joint is missing parent or child")
        if set(parent_element.attrib) != {"link"} or set(child_element.attrib) != {"link"}:
            raise ValueError("proxy joint parent and child must name one link")
        parent = parent_element.attrib["link"]
        child = child_element.attrib["link"]
        if parent not in links or child not in links or parent == child:
            raise ValueError("proxy joint references invalid links")
        if child in child_to_parent:
            raise ValueError("proxy link has more than one parent")
        child_to_parent[child] = parent
        origin = joint.find("origin")
        translation = _parse_vector(
            origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0",
            "joint origin xyz",
        )
        rpy = _parse_vector(
            origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0",
            "joint origin rpy",
        )
        transforms.append(
            JointTransform(
                parent_frame=parent,
                child_frame=child,
                translation=translation,
                rotation=_rpy_to_quaternion(rpy),
            )
        )

    if len(transforms) != len(links) - 1:
        raise ValueError("proxy URDF must be a single link tree")
    roots = set(links) - set(child_to_parent)
    if len(roots) != 1:
        raise ValueError("proxy URDF must have exactly one root link")
    root_frame = next(iter(roots))

    visited = {root_frame}
    changed = True
    while changed:
        changed = False
        for child, parent in child_to_parent.items():
            if parent in visited and child not in visited:
                visited.add(child)
                changed = True
    if visited != set(links):
        raise ValueError("proxy URDF contains a disconnected link or cycle")
    return root_frame, set(links), transforms


def _parse_vector(value: Any, label: str) -> Vector3:
    if not isinstance(value, str):
        raise ValueError(f"{label} must be a string")
    components = value.split()
    if len(components) != 3:
        raise ValueError(f"{label} must have three components")
    return tuple(_strict_float(component, label) for component in components)  # type: ignore


def _rpy_to_quaternion(rpy: Vector3) -> Quaternion:
    half_roll = rpy[0] * 0.5
    half_pitch = rpy[1] * 0.5
    half_yaw = rpy[2] * 0.5
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _validate_name(value: Any, label: str) -> str:
    value = _strict_string(value, label)
    if not _SAFE_NAME.fullmatch(value):
        raise ValueError(f"invalid {label}")
    return value


def _strict_string(value: Any, label: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{label} must be a string")
    return value


def _strict_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} must be an integer")
    return value


def _strict_float(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result
