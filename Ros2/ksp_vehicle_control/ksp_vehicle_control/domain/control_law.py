"""ROS-independent 6DoF setpoint-control calculations."""

from typing import Tuple

from .math3d import (
    Quaternion,
    Vector3,
    add,
    clamp_norm,
    dot,
    norm,
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
    angular_velocity_body: Vector3,
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
    angular_rate_limit: float = 0.0,
) -> Tuple[Vector3, Vector3]:
    force_world = (0.0, 0.0, 0.0)
    if translation_enabled:
        force_world = clamp_norm(
            add(
                scale(subtract(desired_position, position), position_kp),
                scale(subtract(desired_linear_velocity, linear_velocity), velocity_kd),
            ),
            max_force,
        )
    world_to_body = quaternion_conjugate(orientation)
    attitude_error_body = rotate_vector(
        world_to_body,
        quaternion_error_vector(desired_orientation, orientation),
    )
    desired_angular_velocity_body = rotate_vector(
        world_to_body, desired_angular_velocity
    )
    # Treat attitude error as a requested body rate before closing the inner
    # angular-velocity loop.  Clamping that request prevents a large look-at
    # step from pinning torque at its limit until the craft has already spun
    # past the safety rate.  With no rate limit this is algebraically the same
    # PD law as attitude_kp * error + angular_kd * rate_error.
    requested_rate_body = add(
        desired_angular_velocity_body,
        scale(attitude_error_body, attitude_kp / angular_kd),
    )
    if angular_rate_limit > 0.0:
        requested_rate_body = clamp_norm(requested_rate_body, angular_rate_limit)
    torque_body = clamp_norm(
        scale(subtract(requested_rate_body, angular_velocity_body), angular_kd),
        max_torque,
    )
    return rotate_vector(world_to_body, force_world), torque_body


def body_detumble_torque(
    angular_velocity_body: Vector3,
    damping: float,
    maximum: float,
) -> Vector3:
    return clamp_norm(scale(angular_velocity_body, -damping), maximum)


def rate_guard_body_torque(
    torque_body: Vector3,
    angular_velocity_body: Vector3,
    rate_limit: float,
    braking_gain: float,
    maximum: float,
) -> Vector3:
    rate = norm(angular_velocity_body)
    if rate <= rate_limit or rate <= 1.0e-9:
        return torque_body
    rate_axis = scale(angular_velocity_body, 1.0 / rate)
    current_parallel = dot(torque_body, rate_axis)
    required_braking = -min(maximum, braking_gain * (rate - rate_limit))
    if current_parallel > required_braking:
        torque_body = add(torque_body, scale(rate_axis, required_braking - current_parallel))
    return clamp_norm(torque_body, maximum)
