"""Small dependency-free helpers for planar wrench control."""

import math
from typing import Tuple


Vector3 = Tuple[float, float, float]


def clamp(value: float, magnitude: float) -> float:
    """Clamp a scalar symmetrically around zero."""
    limit = abs(float(magnitude))
    return max(-limit, min(limit, float(value)))


def planar_wrench(
    target_velocity: Vector3,
    measured_velocity: Vector3,
    linear_gain: float,
    angular_gain: float,
    max_force: float,
    max_torque: float,
) -> Tuple[Vector3, Vector3]:
    """Compute body-frame x/y force and yaw torque for a planar velocity target."""
    force = (
        clamp((target_velocity[0] - measured_velocity[0]) * linear_gain, max_force),
        clamp((target_velocity[1] - measured_velocity[1]) * linear_gain, max_force),
        0.0,
    )
    torque = (
        0.0,
        0.0,
        clamp((target_velocity[2] - measured_velocity[2]) * angular_gain, max_torque),
    )
    return force, torque


def rotate_planar(vector: Vector3, yaw: float) -> Vector3:
    """Rotate a planar vector by yaw while preserving its third component."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        cosine * vector[0] - sine * vector[1],
        sine * vector[0] + cosine * vector[1],
        vector[2],
    )
