"""Small right-handed 3D math helpers used by the control domain."""

import math
from typing import Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def add(left: Vector3, right: Vector3) -> Vector3:
    return tuple(left[index] + right[index] for index in range(3))  # type: ignore[return-value]


def subtract(left: Vector3, right: Vector3) -> Vector3:
    return tuple(left[index] - right[index] for index in range(3))  # type: ignore[return-value]


def scale(value: Vector3, factor: float) -> Vector3:
    return tuple(component * factor for component in value)  # type: ignore[return-value]


def dot(left: Vector3, right: Vector3) -> float:
    return sum(left[index] * right[index] for index in range(3))


def norm(value: Vector3) -> float:
    return math.sqrt(dot(value, value))


def clamp_norm(value: Vector3, maximum: float) -> Vector3:
    magnitude = norm(value)
    return scale(value, maximum / magnitude) if maximum > 0.0 and magnitude > maximum else value


def quaternion_conjugate(value: Quaternion) -> Quaternion:
    return (-value[0], -value[1], -value[2], value[3])


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_normalize(value: Quaternion) -> Quaternion:
    magnitude = math.sqrt(sum(component * component for component in value))
    if magnitude <= 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / magnitude for component in value)  # type: ignore[return-value]


def rotate_vector(rotation: Quaternion, value: Vector3) -> Vector3:
    normalized = quaternion_normalize(rotation)
    rotated = quaternion_multiply(
        quaternion_multiply(normalized, (value[0], value[1], value[2], 0.0)),
        quaternion_conjugate(normalized),
    )
    return (rotated[0], rotated[1], rotated[2])


def quaternion_error_vector(desired: Quaternion, current: Quaternion) -> Vector3:
    error = quaternion_normalize(
        quaternion_multiply(desired, quaternion_conjugate(current))
    )
    if error[3] < 0.0:
        error = tuple(-value for value in error)  # type: ignore[assignment]
    vector = (error[0], error[1], error[2])
    sine = norm(vector)
    if sine <= 1.0e-12:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(sine, max(0.0, error[3]))
    return scale(vector, angle / sine)


def linear_ramp_fraction(elapsed: float, duration: float) -> float:
    if not math.isfinite(elapsed) or elapsed <= 0.0:
        return 0.0
    if not math.isfinite(duration) or duration <= 0.0:
        return 1.0
    return min(1.0, elapsed / duration)
