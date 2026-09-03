"""ROS-independent 6DoF setpoint-control calculations."""

from __future__ import annotations

from typing import Tuple

from .core import (
    Quaternion,
    Vector3,
    add,
    clamp_norm,
    quaternion_conjugate,
    quaternion_error_vector,
    rotate_vector,
    scale,
    subtract,
)


def body_wrench_for_setpoint(
    position: Vector3,
    orientation: Quaternion,
    linear_velocity: Vector3,
    angular_velocity: Vector3,
    desired_position: Vector3,
    desired_orientation: Quaternion,
    desired_linear_velocity: Vector3,
    desired_angular_velocity: Vector3,
    position_kp: float,
    velocity_kd: float,
    attitude_kp: float,
    angular_kd: float,
    max_force: float,
    max_torque: float,
    translation_enabled: bool,
) -> Tuple[Vector3, Vector3]:
    """Calculate a saturated body-frame wrench from a world-frame setpoint."""
    force_world = (0.0, 0.0, 0.0)
    if translation_enabled:
        force_world = add(
            scale(subtract(desired_position, position), position_kp),
            scale(subtract(desired_linear_velocity, linear_velocity), velocity_kd),
        )
        force_world = clamp_norm(force_world, max_force)
    torque_world = add(
        scale(quaternion_error_vector(desired_orientation, orientation), attitude_kp),
        scale(subtract(desired_angular_velocity, angular_velocity), angular_kd),
    )
    torque_world = clamp_norm(torque_world, max_torque)
    world_to_body = quaternion_conjugate(orientation)
    return (
        rotate_vector(world_to_body, force_world),
        rotate_vector(world_to_body, torque_world),
    )


def body_detumble_torque(
    orientation: Quaternion,
    angular_velocity: Vector3,
    damping: float,
    maximum: float,
) -> Vector3:
    """Calculate a saturated body-frame torque opposite world angular velocity."""
    torque_world = clamp_norm(scale(angular_velocity, -damping), maximum)
    return rotate_vector(quaternion_conjugate(orientation), torque_world)
