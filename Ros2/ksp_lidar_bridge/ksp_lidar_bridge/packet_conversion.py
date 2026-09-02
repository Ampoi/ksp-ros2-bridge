import json
import math
import re
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
_GOLDEN_ANGLE_RADIANS = 2.3999632


@dataclass(frozen=True)
class LaserScanData:
    horizontal_count: int
    angle_min: float
    angle_max: float
    angle_increment: float
    scan_time: float
    time_increment: float
    range_min: float
    range_max: float
    ranges: List[float]


@dataclass(frozen=True)
class SensorPose:
    translation: Vector3
    rotation: Quaternion


def decode_datagram(data: bytes) -> Dict[str, Any]:
    packet = json.loads(data.decode("utf-8"))
    if not isinstance(packet, dict):
        raise ValueError("packet root must be a JSON object")
    return packet


def expired_topic_names(
    last_seen_by_topic: Mapping[str, float], now: float, timeout: float
) -> List[str]:
    return [
        topic
        for topic, last_seen in last_seen_by_topic.items()
        if now - last_seen >= timeout
    ]


def sanitize_ros_name(value: Any, fallback: str = "lidar") -> str:
    name = re.sub(r"[^A-Za-z0-9_]", "_", str(value or ""))
    name = re.sub(r"_+", "_", name).strip("_").lower()
    if not name:
        name = fallback
    if not re.match(r"^[A-Za-z_]", name):
        name = "_" + name
    return name


def as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (OverflowError, TypeError, ValueError):
        return default


def as_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (OverflowError, TypeError, ValueError):
        return default


def packet_part_name(packet: Dict[str, Any]) -> str:
    explicit = packet.get("partName") or packet.get("lidarName") or packet.get("name")
    if explicit:
        return str(explicit)

    vessel = sanitize_ros_name(packet.get("vessel"), "vessel")
    part_id = as_int(packet.get("partFlightId"), 0)
    return f"{vessel}_{part_id}"


def packet_lidar_name(packet: Dict[str, Any]) -> str:
    """Backward-compatible alias for callers using the old sensor-specific name."""
    return packet_part_name(packet)


def lidar_topic_suffix(mode: Any) -> str:
    normalized_mode = str(mode or "").upper()
    if normalized_mode == "2D":
        return "lidar/scan"
    if normalized_mode == "3D":
        return "lidar/points"
    raise ValueError(f"unsupported LiDAR mode: {normalized_mode}")


def lidar_topic_from_packet(packet: Dict[str, Any], topic_prefix: str = "/ros2_ksp") -> str:
    prefix_value = str(topic_prefix or "").strip("/")
    prefix = f"/{prefix_value}" if prefix_value else ""
    part_name = sanitize_ros_name(packet_part_name(packet), "lidar")
    return f"{prefix}/{part_name}/{lidar_topic_suffix(packet.get('mode'))}"


def normalized_ranges(values: Any, count: int, range_max: float) -> List[float]:
    source = values if isinstance(values, list) else []
    ranges: List[float] = []
    for index in range(count):
        raw = source[index] if index < len(source) else math.inf
        value = as_float(raw, math.inf)
        if not math.isfinite(value) or value < 0.0 or value > range_max:
            ranges.append(math.inf)
        else:
            ranges.append(value)
    return ranges


def _vector_from_components(values: Any) -> Vector3:
    if not isinstance(values, (list, tuple)) or len(values) < 3:
        return (math.nan, math.nan, math.nan)
    return (
        as_float(values[0], math.nan),
        as_float(values[1], math.nan),
        as_float(values[2], math.nan),
    )


def chunked_vectors(values: Any) -> Iterable[Vector3]:
    """Yield vectors from flat KSP arrays or legacy nested vector arrays."""
    if not isinstance(values, list) or not values:
        return

    if any(isinstance(value, (list, tuple)) for value in values):
        for value in values:
            yield _vector_from_components(value)
        return

    for index in range(0, len(values), 3):
        yield _vector_from_components(values[index : index + 3])


def points_from_packet(packet: Dict[str, Any]) -> List[Vector3]:
    range_max = max(0.001, as_float(packet.get("maxDistance"), 0.001))
    ray_count = max(0, as_int(packet.get("rayCount"), 0))
    ranges = normalized_ranges(packet.get("ranges"), ray_count, range_max)

    points = packet.get("points")
    if isinstance(points, list) and points:
        output = []
        for index, point in enumerate(chunked_vectors(points)):
            if index >= len(ranges):
                break
            if math.isfinite(ranges[index]) and all(math.isfinite(axis) for axis in point):
                output.append(point)
        return output

    directions = list(chunked_vectors(packet.get("directions")))
    if (
        not directions
        and packet.get("coordinateFrame") == "ros_sensor"
        and packet.get("layout") == "fibonacci-hemisphere"
    ):
        directions = fibonacci_hemisphere_directions(ray_count)
    output = []
    for index, distance in enumerate(ranges):
        if index >= len(directions):
            break
        direction = directions[index]
        if not math.isfinite(distance) or not all(math.isfinite(axis) for axis in direction):
            continue
        output.append(
            (
                direction[0] * distance,
                direction[1] * distance,
                direction[2] * distance,
            )
        )
    return output


def fibonacci_hemisphere_directions(count: int) -> List[Vector3]:
    """Recreate KSP's compact 3D layout in the canonical ROS sensor frame."""
    if count <= 0:
        return []
    if count == 1:
        return [(1.0, 0.0, 0.0)]

    directions: List[Vector3] = []
    for index in range(count):
        forward = (index + 0.5) / count
        radius = math.sqrt(max(0.0, 1.0 - forward * forward))
        azimuth = index * _GOLDEN_ANGLE_RADIANS
        directions.append(
            (
                forward,
                -math.cos(azimuth) * radius,
                math.sin(azimuth) * radius,
            )
        )
    return directions


def sensor_pose_from_packet(packet: Dict[str, Any]) -> Optional[SensorPose]:
    """Read the ROS sensor frame pose relative to the owning URDF part link."""
    if packet.get("coordinateFrame") != "ros_sensor":
        return None

    position = packet.get("framePosition")
    rotation = packet.get("frameRotation")
    if not isinstance(position, (list, tuple)) or len(position) != 3:
        return None
    if not isinstance(rotation, (list, tuple)) or len(rotation) != 4:
        return None

    translation = tuple(as_float(value, math.nan) for value in position)
    quaternion = tuple(as_float(value, math.nan) for value in rotation)
    if not all(math.isfinite(value) for value in translation + quaternion):
        return None

    magnitude = math.sqrt(sum(value * value for value in quaternion))
    if magnitude < 1e-9:
        return None
    normalized = tuple(value / magnitude for value in quaternion)
    return SensorPose(
        translation=translation,  # type: ignore[arg-type]
        rotation=normalized,  # type: ignore[arg-type]
    )


def laser_scan_from_packet(packet: Dict[str, Any]) -> LaserScanData:
    horizontal_count = max(1, as_int(packet.get("horizontalCount"), 1))
    range_max = max(0.001, as_float(packet.get("maxDistance"), 0.001))
    fov_rad = math.radians(max(0.0, as_float(packet.get("horizontalFovDeg"), 0.0)))
    if fov_rad <= 0.0:
        fov_rad = math.tau

    angle_min = -fov_rad * 0.5
    if abs(math.degrees(fov_rad)) >= 359.9:
        angle_increment = fov_rad / horizontal_count
    else:
        angle_increment = fov_rad / max(horizontal_count - 1, 1)

    scan_rate = as_float(packet.get("scanRateHz"), 1.0)
    scan_time = 1.0 / scan_rate if scan_rate > 0.0 else 0.0
    return LaserScanData(
        horizontal_count=horizontal_count,
        angle_min=angle_min,
        angle_max=angle_min + angle_increment * max(horizontal_count - 1, 0),
        angle_increment=angle_increment,
        scan_time=scan_time,
        time_increment=scan_time / horizontal_count,
        range_min=0.0,
        range_max=range_max,
        ranges=normalized_ranges(packet.get("ranges"), horizontal_count, range_max),
    )
