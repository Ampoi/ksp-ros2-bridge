"""ROS-independent geometry, clustering, and image helpers for the demo."""

from __future__ import annotations

import binascii
import math
import re
import struct
import zlib
from collections import deque
from typing import Iterable, List, NamedTuple, Optional, Sequence, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


class VesselTopics(NamedTuple):
    """Topic names used by this demo and the active-vessel read model."""

    lidar_points: str
    camera_image: str
    imu_data: str
    control_setpoint: str
    controller_status: str
    demo_status: str


def sanitize_ros_component(value: str, fallback: str) -> str:
    """Mirror the bridge's normalization of user-configurable name tokens."""
    name = re.sub(r"[^A-Za-z0-9_]", "_", str(value or ""))
    name = re.sub(r"_+", "_", name).strip("_").lower()
    if not name:
        name = fallback
    if not re.match(r"^[A-Za-z_]", name):
        name = "_" + name
    return name


def vessel_topics(
    prefix: str, lidar_sensor_id: str, camera_sensor_id: str, demo_instance_id: str
) -> VesselTopics:
    """Resolve sensor inputs and demo-owned output topics.

    ``demo_instance_id`` is only a ROS namespace token.  It never selects a KSP
    vessel; control adapters bind commands to lifecycle ``vessel_id`` values.
    """
    root_value = str(prefix or "").strip("/")
    root = f"/{root_value}" if root_value else ""
    lidar_id = sanitize_ros_component(lidar_sensor_id, "lidar_3d")
    camera_id = sanitize_ros_component(camera_sensor_id, "camera")
    instance = sanitize_ros_component(demo_instance_id, "demo_vehicle")
    return VesselTopics(
        lidar_points=f"{root}/lidar_3d/{lidar_id}/points",
        camera_image=f"{root}/camera/{camera_id}/image_raw",
        imu_data=f"{root}/imu/data_raw",
        control_setpoint=f"{root}/demos/debris_orbit/{instance}/setpoint",
        controller_status=f"{root}/demos/debris_orbit/{instance}/controller_status",
        demo_status=f"{root}/demos/debris_orbit/{instance}/status",
    )


def safe_filename_component(value: str, fallback: str = "unnamed") -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("._-")
    return cleaned or fallback


def add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def subtract(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Vector3, value: float) -> Vector3:
    return (a[0] * value, a[1] * value, a[2] * value)


def predict_linear_at(
    position: Vector3,
    velocity: Vector3,
    sample_time_sec: float,
    reference_time_sec: float,
) -> Vector3:
    """Predict at an explicit data timestamp, never implicitly at wall clock."""
    elapsed = max(0.0, reference_time_sec - sample_time_sec)
    return add(position, scale(velocity, elapsed))


def dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a: Vector3) -> float:
    return math.sqrt(dot(a, a))


def normalize(a: Vector3, fallback: Optional[Vector3] = None) -> Vector3:
    length = norm(a)
    if length > 1.0e-9:
        return scale(a, 1.0 / length)
    if fallback is not None:
        return normalize(fallback)
    raise ValueError("cannot normalize a zero-length vector")


def clamp_norm(a: Vector3, maximum: float) -> Vector3:
    length = norm(a)
    if maximum > 0.0 and length > maximum:
        return scale(a, maximum / length)
    return a


def linear_ramp_fraction(elapsed: float, duration: float) -> float:
    """Return a bounded 0..1 command ramp, including safe invalid-input handling."""
    if not math.isfinite(elapsed) or elapsed <= 0.0:
        return 0.0
    if not math.isfinite(duration) or duration <= 0.0:
        return 1.0
    return min(1.0, elapsed / duration)


def detumble_required(
    active: bool, angular_rate: float, enter_rate: float, exit_rate: float
) -> bool:
    """Apply hysteresis to the detumble mode transition."""
    threshold = exit_rate if active else enter_rate
    return math.isfinite(angular_rate) and angular_rate > threshold


def quaternion_conjugate(q: Quaternion) -> Quaternion:
    return (-q[0], -q[1], -q[2], q[3])


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_normalize(q: Quaternion) -> Quaternion:
    length = math.sqrt(sum(value * value for value in q))
    if length <= 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / length for value in q)  # type: ignore[return-value]


def quaternion_from_rotation_vector(vector: Vector3) -> Quaternion:
    """Convert an axis-angle vector in radians to a quaternion."""
    angle = norm(vector)
    if angle < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    half_angle = 0.5 * angle
    factor = math.sin(half_angle) / angle
    return (
        vector[0] * factor,
        vector[1] * factor,
        vector[2] * factor,
        math.cos(half_angle),
    )


def integrate_world_orientation(
    orientation: Quaternion, angular_velocity: Vector3, elapsed: float
) -> Quaternion:
    """Move an orientation by a world-frame angular velocity for `elapsed` seconds."""
    delta = quaternion_from_rotation_vector(scale(angular_velocity, elapsed))
    return quaternion_normalize(quaternion_multiply(delta, orientation))


def rotate_vector(q: Quaternion, vector: Vector3) -> Vector3:
    qn = quaternion_normalize(q)
    vector_q = (vector[0], vector[1], vector[2], 0.0)
    rotated = quaternion_multiply(
        quaternion_multiply(qn, vector_q), quaternion_conjugate(qn)
    )
    return (rotated[0], rotated[1], rotated[2])


def matrix_to_quaternion(
    x_axis: Vector3, y_axis: Vector3, z_axis: Vector3
) -> Quaternion:
    """Convert a rotation matrix expressed as three world-space columns."""
    m00, m10, m20 = x_axis
    m01, m11, m21 = y_axis
    m02, m12, m22 = z_axis
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        q = (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        q = ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        q = ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)
    return quaternion_normalize(q)


def look_at_quaternion(forward: Vector3, up_hint: Vector3) -> Quaternion:
    """Return orientation for a body whose +X faces forward and +Z faces up."""
    x_axis = normalize(forward, (1.0, 0.0, 0.0))
    y_axis = cross(up_hint, x_axis)
    if norm(y_axis) < 1.0e-6:
        alternate = (0.0, 1.0, 0.0) if abs(x_axis[1]) < 0.9 else (0.0, 0.0, 1.0)
        y_axis = cross(alternate, x_axis)
    y_axis = normalize(y_axis)
    z_axis = normalize(cross(x_axis, y_axis))
    return matrix_to_quaternion(x_axis, y_axis, z_axis)


def body_orientation_for_sensor_look_at(
    forward_world: Vector3,
    up_hint_world: Vector3,
    sensor_in_body: Quaternion,
) -> Quaternion:
    """Point sensor +X at a world direction while compensating its mounting rotation."""
    sensor_world = look_at_quaternion(forward_world, up_hint_world)
    return quaternion_normalize(
        quaternion_multiply(sensor_world, quaternion_conjugate(sensor_in_body))
    )


def quaternion_between_vectors(source: Vector3, target: Vector3) -> Quaternion:
    """Return the shortest rotation that moves one direction onto another."""
    source_unit = normalize(source, (1.0, 0.0, 0.0))
    target_unit = normalize(target, source_unit)
    cosine = max(-1.0, min(1.0, dot(source_unit, target_unit)))
    if cosine < -1.0 + 1.0e-8:
        helper = (1.0, 0.0, 0.0) if abs(source_unit[0]) < 0.9 else (0.0, 1.0, 0.0)
        axis = normalize(cross(source_unit, helper), (0.0, 0.0, 1.0))
        return (axis[0], axis[1], axis[2], 0.0)
    axis = cross(source_unit, target_unit)
    return quaternion_normalize((axis[0], axis[1], axis[2], 1.0 + cosine))


def body_orientation_for_sensor_direction(
    current_body_world: Quaternion,
    sensor_in_body: Quaternion,
    forward_world: Vector3,
) -> Quaternion:
    """Aim sensor +X using the minimum rotation, preserving line-of-sight roll."""
    sensor_world = quaternion_multiply(current_body_world, sensor_in_body)
    current_forward = rotate_vector(sensor_world, (1.0, 0.0, 0.0))
    correction = quaternion_between_vectors(current_forward, forward_world)
    return quaternion_normalize(quaternion_multiply(correction, current_body_world))


def search_direction(
    center_direction: Vector3,
    up_hint: Vector3,
    elapsed: float,
    period: float,
    yaw_amplitude: float,
    pitch_amplitude: float,
) -> Vector3:
    """Return a bounded, periodic scan direction around the last line of sight."""
    center = normalize(center_direction, (1.0, 0.0, 0.0))
    side = cross(up_hint, center)
    if norm(side) < 1.0e-6:
        side = cross((0.0, 1.0, 0.0), center)
    side = normalize(side, (0.0, 1.0, 0.0))
    local_up = normalize(cross(center, side), (0.0, 0.0, 1.0))
    phase = 2.0 * math.pi * max(0.0, elapsed) / max(1.0e-6, period)
    yaw = yaw_amplitude * math.sin(phase)
    pitch = pitch_amplitude * math.sin(2.0 * phase)
    return add(scale(add(scale(center, math.cos(yaw)), scale(side, math.sin(yaw))),
                     math.cos(pitch)), scale(local_up, math.sin(pitch)))


def quaternion_error_vector(desired: Quaternion, current: Quaternion) -> Vector3:
    """Shortest world-frame axis-angle error from current to desired."""
    error = quaternion_normalize(
        quaternion_multiply(desired, quaternion_conjugate(quaternion_normalize(current)))
    )
    if error[3] < 0.0:
        error = tuple(-value for value in error)  # type: ignore[assignment]
    vector_length = norm((error[0], error[1], error[2]))
    if vector_length < 1.0e-9:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vector_length, max(0.0, error[3]))
    return scale((error[0], error[1], error[2]), angle / vector_length)


def transform_point(translation: Vector3, rotation: Quaternion, point: Vector3) -> Vector3:
    return add(translation, rotate_vector(rotation, point))


def voxel_downsample(
    points: Iterable[Vector3], voxel_size: float, max_points: int
) -> List[Vector3]:
    """Keep one finite point per voxel, bounded for predictable callback time."""
    selected = {}
    size = max(1.0e-6, voxel_size)
    for point in points:
        if not all(math.isfinite(value) for value in point):
            continue
        key = tuple(math.floor(value / size) for value in point)
        if key not in selected:
            selected[key] = point
            if len(selected) >= max_points:
                break
    return list(selected.values())


def euclidean_clusters(
    points: Sequence[Vector3], tolerance: float, minimum_points: int
) -> List[List[Vector3]]:
    """Cluster points using a 3D hash grid and neighboring-cell searches."""
    if not points:
        return []
    cell_size = max(1.0e-6, tolerance)
    cells = {}
    for index, point in enumerate(points):
        key = tuple(math.floor(value / cell_size) for value in point)
        cells.setdefault(key, []).append(index)

    tolerance_sq = tolerance * tolerance
    unseen = set(range(len(points)))
    clusters: List[List[Vector3]] = []
    while unseen:
        seed = unseen.pop()
        members = [seed]
        queue = deque([seed])
        while queue:
            current = queue.popleft()
            point = points[current]
            cell = tuple(math.floor(value / cell_size) for value in point)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        for candidate in cells.get(
                            (cell[0] + dx, cell[1] + dy, cell[2] + dz), ()
                        ):
                            if candidate not in unseen:
                                continue
                            delta = subtract(points[candidate], point)
                            if dot(delta, delta) <= tolerance_sq:
                                unseen.remove(candidate)
                                members.append(candidate)
                                queue.append(candidate)
        if len(members) >= minimum_points:
            clusters.append([points[index] for index in members])
    clusters.sort(key=len, reverse=True)
    return clusters


def bounding_box_center(points: Sequence[Vector3]) -> Vector3:
    if not points:
        raise ValueError("a cluster must contain at least one point")
    return tuple(
        (min(point[axis] for point in points) + max(point[axis] for point in points)) * 0.5
        for axis in range(3)
    )  # type: ignore[return-value]


def make_orbit_basis(
    radial: Vector3, normal: Vector3, direction: int
) -> Tuple[Vector3, Vector3, Vector3]:
    normal_unit = normalize(normal, (0.0, 0.0, 1.0))
    projected = subtract(radial, scale(normal_unit, dot(radial, normal_unit)))
    if norm(projected) < 1.0e-6:
        projected = cross(normal_unit, (1.0, 0.0, 0.0))
        if norm(projected) < 1.0e-6:
            projected = cross(normal_unit, (0.0, 1.0, 0.0))
    u_axis = normalize(projected)
    v_axis = scale(normalize(cross(normal_unit, u_axis)), 1.0 if direction >= 0 else -1.0)
    return u_axis, v_axis, normal_unit


def unwrap_angle(previous: float, wrapped: float) -> float:
    delta = (wrapped - previous + math.pi) % (2.0 * math.pi) - math.pi
    return previous + delta


def png_bytes(width: int, height: int, encoding: str, step: int, data: bytes) -> bytes:
    """Encode ROS rgb8/bgr8/mono8 pixels as a dependency-free RGB PNG."""
    channels = 1 if encoding == "mono8" else 3
    if encoding not in ("rgb8", "bgr8", "mono8"):
        raise ValueError(f"unsupported image encoding: {encoding}")
    row_bytes = width * channels
    if width <= 0 or height <= 0 or step < row_bytes or len(data) < step * height:
        raise ValueError("invalid image dimensions or data length")
    raw = bytearray()
    for row_index in range(height):
        row = data[row_index * step : row_index * step + row_bytes]
        raw.append(0)
        if encoding == "rgb8":
            raw.extend(row)
        elif encoding == "bgr8":
            for offset in range(0, len(row), 3):
                raw.extend((row[offset + 2], row[offset + 1], row[offset]))
        else:
            for value in row:
                raw.extend((value, value, value))

    def chunk(name: bytes, payload: bytes) -> bytes:
        body = name + payload
        checksum = struct.pack(">I", binascii.crc32(body) & 0xFFFFFFFF)
        return struct.pack(">I", len(payload)) + body + checksum

    signature = b"\x89PNG\r\n\x1a\n"
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (
        signature
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
        + chunk(b"IEND", b"")
    )
