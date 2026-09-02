import base64
import binascii
import hashlib
import math
import re
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


_HEX_32 = re.compile(r"^[0-9a-f]{32}$")
_HEX_64 = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True)
class CameraFrame:
    source: str
    part_name: str
    part_flight_id: int
    universal_time: float
    width: int
    height: int
    step: int
    encoding: str
    vertical_fov_degrees: float
    frame_position: Tuple[float, float, float]
    frame_rotation: Tuple[float, float, float, float]
    data: bytes


@dataclass
class _PendingFrame:
    metadata: Tuple[Any, ...]
    chunks: List[Optional[bytes]]
    received_bytes: int
    updated_at: float


class CameraFrameAssembler:
    """Reassemble bounded, checksummed raw RGB frames from UDP JSON chunks."""

    def __init__(
        self,
        max_frame_bytes: int = 4 * 1024 * 1024,
        max_chunks: int = 1024,
        max_pending_frames: int = 16,
        timeout_sec: float = 2.0,
    ) -> None:
        self.max_frame_bytes = max_frame_bytes
        self.max_chunks = max_chunks
        self.max_pending_frames = max_pending_frames
        self.timeout_sec = timeout_sec
        self._pending: Dict[Tuple[str, int, int], _PendingFrame] = {}

    def consume(self, packet: Dict[str, Any], now: Optional[float] = None) -> Optional[CameraFrame]:
        timestamp = time.monotonic() if now is None else now
        self.expire(timestamp)
        values = self._validate(packet)
        (
            key,
            chunk_index,
            chunk_count,
            frame_bytes,
            checksum,
            metadata,
            chunk,
        ) = values

        pending = self._pending.get(key)
        if pending is None:
            if len(self._pending) >= self.max_pending_frames:
                oldest_key = min(self._pending, key=lambda item: self._pending[item].updated_at)
                del self._pending[oldest_key]
            pending = _PendingFrame(
                metadata=metadata,
                chunks=[None] * chunk_count,
                received_bytes=0,
                updated_at=timestamp,
            )
            self._pending[key] = pending
        elif pending.metadata != metadata or len(pending.chunks) != chunk_count:
            del self._pending[key]
            raise ValueError("camera frame metadata changed between chunks")

        previous = pending.chunks[chunk_index]
        if previous is not None:
            if previous != chunk:
                del self._pending[key]
                raise ValueError("camera frame chunk was replaced with different data")
            pending.updated_at = timestamp
            return None

        pending.chunks[chunk_index] = chunk
        pending.received_bytes += len(chunk)
        pending.updated_at = timestamp
        if pending.received_bytes > frame_bytes:
            del self._pending[key]
            raise ValueError("camera chunks exceed declared frame size")
        if any(value is None for value in pending.chunks):
            return None

        payload = b"".join(value for value in pending.chunks if value is not None)
        del self._pending[key]
        if len(payload) != frame_bytes:
            raise ValueError("camera frame size does not match its chunks")
        if hashlib.sha256(payload).hexdigest() != checksum:
            raise ValueError("camera frame checksum mismatch")

        (
            source,
            part_name,
            part_flight_id,
            universal_time,
            width,
            height,
            step,
            encoding,
            vertical_fov,
            frame_position,
            frame_rotation,
            _checksum,
            _frame_bytes,
        ) = metadata
        return CameraFrame(
            source=source,
            part_name=part_name,
            part_flight_id=part_flight_id,
            universal_time=universal_time,
            width=width,
            height=height,
            step=step,
            encoding=encoding,
            vertical_fov_degrees=vertical_fov,
            frame_position=frame_position,
            frame_rotation=frame_rotation,
            data=payload,
        )

    def expire(self, now: Optional[float] = None) -> None:
        timestamp = time.monotonic() if now is None else now
        stale = [
            key
            for key, pending in self._pending.items()
            if timestamp - pending.updated_at >= self.timeout_sec
        ]
        for key in stale:
            del self._pending[key]

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    def _validate(self, packet: Dict[str, Any]) -> Tuple[Any, ...]:
        if packet.get("type") != "ksp_camera_frame_chunk" or packet.get("version") != 1:
            raise ValueError("unsupported camera chunk type or version")

        session_id = packet.get("sessionId")
        checksum = packet.get("sha256")
        if not isinstance(session_id, str) or not _HEX_32.fullmatch(session_id):
            raise ValueError("invalid camera session ID")
        if not isinstance(checksum, str) or not _HEX_64.fullmatch(checksum):
            raise ValueError("invalid camera frame checksum")

        sequence = _strict_int(packet.get("sequence"), "sequence", 1, 2**63 - 1)
        chunk_index = _strict_int(packet.get("chunkIndex"), "chunk index", 0, self.max_chunks - 1)
        chunk_count = _strict_int(packet.get("chunkCount"), "chunk count", 1, self.max_chunks)
        if chunk_index >= chunk_count:
            raise ValueError("camera chunk index is outside the frame")
        frame_bytes = _strict_int(packet.get("frameBytes"), "frame size", 1, self.max_frame_bytes)
        width = _strict_int(packet.get("width"), "image width", 1, 4096)
        height = _strict_int(packet.get("height"), "image height", 1, 4096)
        step = _strict_int(packet.get("step"), "image step", 1, 65536)
        encoding = packet.get("encoding")
        if encoding != "rgb8" or step != width * 3 or frame_bytes != step * height:
            raise ValueError("camera frame must be tightly packed rgb8")

        encoded = packet.get("data")
        if not isinstance(encoded, str):
            raise ValueError("camera chunk data must be base64 text")
        try:
            chunk = base64.b64decode(encoded, validate=True)
        except (binascii.Error, ValueError) as exc:
            raise ValueError("invalid camera chunk base64") from exc
        if not chunk or len(chunk) > min(self.max_frame_bytes, 65536):
            raise ValueError("camera chunk has an invalid decoded size")

        source = str(packet.get("source") or "rgb_camera")
        if source not in ("rgb_camera", "docking_port"):
            raise ValueError("invalid camera source")
        part_name = str(packet.get("partName") or "rgb_camera")
        part_flight_id = _strict_int(packet.get("partFlightId", 0), "part flight ID", 0, 2**64 - 1)
        universal_time = _finite_float(packet.get("universalTime"), "universal time")
        vertical_fov = _finite_float(packet.get("verticalFovDeg"), "vertical FOV")
        if vertical_fov <= 0.0 or vertical_fov >= 180.0:
            raise ValueError("camera vertical FOV is outside its valid range")
        frame_position = _vector(packet.get("framePosition"), 3, "frame position")
        frame_rotation = _vector(packet.get("frameRotation"), 4, "frame rotation")
        rotation_length = math.sqrt(sum(value * value for value in frame_rotation))
        if rotation_length < 1e-9:
            raise ValueError("camera frame rotation is invalid")
        frame_rotation = tuple(value / rotation_length for value in frame_rotation)

        key = (session_id, sequence, part_flight_id, source, part_name)
        metadata = (
            source,
            part_name,
            part_flight_id,
            universal_time,
            width,
            height,
            step,
            encoding,
            vertical_fov,
            frame_position,
            frame_rotation,
            checksum,
            frame_bytes,
        )
        return key, chunk_index, chunk_count, frame_bytes, checksum, metadata, chunk


def camera_topics(
    part_name: Any,
    topic_prefix: str = "/ros2_ksp",
    source: str = "rgb_camera",
    docking_ports_prefix: Optional[str] = None,
) -> Tuple[str, str]:
    from .packet_conversion import sanitize_ros_name

    prefix_value = str(topic_prefix or "").strip("/")
    prefix = f"/{prefix_value}" if prefix_value else ""
    fallback = "docking_port" if source == "docking_port" else "rgb_camera"
    name = sanitize_ros_name(part_name, fallback)
    if source == "docking_port":
        docking_prefix = (
            "/" + str(docking_ports_prefix).strip("/")
            if docking_ports_prefix is not None
            else f"{prefix}/docking_ports"
        )
        base = f"{docking_prefix}/{name}/camera"
    elif source == "rgb_camera":
        base = f"{prefix}/{name}/camera"
    else:
        raise ValueError("unsupported camera source")
    return f"{base}/image_raw", f"{base}/camera_info"


def camera_optical_rotation(
    sensor_rotation: Tuple[float, float, float, float]
) -> Tuple[float, float, float, float]:
    """Rotate ROS +X-forward/+Z-up sensor axes into REP-103 camera optical axes."""
    x1, y1, z1, w1 = sensor_rotation
    x2, y2, z2, w2 = (-0.5, 0.5, -0.5, 0.5)
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _strict_int(value: Any, label: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool):
        raise ValueError(f"invalid camera {label}")
    try:
        parsed = int(value)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid camera {label}") from exc
    if parsed < minimum or parsed > maximum:
        raise ValueError(f"camera {label} is outside its limit")
    return parsed


def _finite_float(value: Any, label: str) -> float:
    try:
        parsed = float(value)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid camera {label}") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"invalid camera {label}")
    return parsed


def _vector(value: Any, length: int, label: str) -> Tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"invalid camera {label}")
    parsed = tuple(_finite_float(component, label) for component in value)
    return parsed
