"""Simulation-time mapping and constant-velocity pose extrapolation."""

import math
from typing import Tuple


Quaternion = Tuple[float, float, float, float]
Vector3 = Tuple[float, float, float]


class SimulationClock:
    """Map one flight session using a fixed simulation-to-ROS time offset."""

    def __init__(self) -> None:
        self._offset_sec = None

    def reset(self) -> None:
        self._offset_sec = None

    def map_nanoseconds(self, universal_time: float, receipt_nanoseconds: int) -> int:
        if not math.isfinite(universal_time):
            return receipt_nanoseconds
        receipt_sec = receipt_nanoseconds * 1.0e-9
        candidate = receipt_sec - universal_time
        if self._offset_sec is None:
            self._offset_sec = candidate
        predicted = universal_time + self._offset_sec
        return int(round(predicted * 1.0e9))


def extrapolate_pose(
    position: Vector3,
    rotation: Quaternion,
    linear_velocity: Vector3,
    angular_velocity: Vector3,
    elapsed: float,
) -> Tuple[Vector3, Quaternion]:
    bounded_elapsed = max(-0.1, min(0.1, elapsed))
    translated = tuple(
        position[index] + linear_velocity[index] * bounded_elapsed for index in range(3)
    )
    speed = math.sqrt(sum(value * value for value in angular_velocity))
    if speed <= 1.0e-12 or abs(bounded_elapsed) <= 1.0e-12:
        return translated, _normalize(rotation)  # type: ignore[return-value]
    angle = speed * bounded_elapsed
    sine = math.sin(angle * 0.5) / speed
    delta = (
        angular_velocity[0] * sine,
        angular_velocity[1] * sine,
        angular_velocity[2] * sine,
        math.cos(angle * 0.5),
    )
    return translated, _normalize(_multiply(delta, rotation))  # type: ignore[return-value]


def _multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _normalize(value: Quaternion) -> Quaternion:
    magnitude = math.sqrt(sum(component * component for component in value))
    if magnitude <= 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / magnitude for component in value)  # type: ignore[return-value]
