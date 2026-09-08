"""ROS-independent helpers for 6DoF scan-to-scan 3D LiDAR odometry."""

from __future__ import annotations

import math
import re
from typing import Iterable, Tuple

import numpy as np
from pylon_perception.points import filter_range, voxel_downsample, transform_points


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def sanitize_ros_component(value: str, fallback: str) -> str:
    """Mirror the bridge's normalization of user-configurable name tokens."""
    name = re.sub(r"[^A-Za-z0-9_]", "_", str(value or ""))
    name = re.sub(r"_+", "_", name).strip("_").lower()
    if not name:
        name = fallback
    if not re.match(r"^[A-Za-z_]", name):
        name = "_" + name
    return name


def default_lidar_topic(prefix: str, lidar_sensor_id: str) -> str:
    """Resolve the default 3D LiDAR topic for the `/ksp_vessel` API."""
    root_value = str(prefix or "").strip("/")
    root = f"/{root_value}" if root_value else ""
    lidar_id = sanitize_ros_component(lidar_sensor_id, "lidar_3d")
    return f"{root}/lidar_3d/{lidar_id}/points"


def default_ground_truth_pose_topic(prefix: str) -> str:
    """Resolve the default ground-truth pose topic for the `/ksp_vessel` API."""
    root_value = str(prefix or "").strip("/")
    root = f"/{root_value}" if root_value else ""
    return f"{root}/ground_truth/pose"








def invert_transform(transform: np.ndarray) -> np.ndarray:
    """Invert a homogeneous SE(3) transform."""
    matrix = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    rotation = matrix[:3, :3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ matrix[:3, 3]
    return inverse


def nearest_neighbors(
    source: np.ndarray, target: np.ndarray, chunk_size: int = 256
) -> Tuple[np.ndarray, np.ndarray]:
    """Find a target neighbor index and distance for each source point."""
    source_array = np.asarray(source, dtype=np.float64).reshape(-1, 3)
    target_array = np.asarray(target, dtype=np.float64).reshape(-1, 3)
    count = source_array.shape[0]
    indices = np.empty((count,), dtype=np.int64)
    distances = np.empty((count,), dtype=np.float64)
    step = max(1, int(chunk_size))
    for start in range(0, count, step):
        block = source_array[start : start + step]
        differences = block[:, np.newaxis, :] - target_array[np.newaxis, :, :]
        squared = np.einsum("ijk,ijk->ij", differences, differences)
        block_indices = np.argmin(squared, axis=1)
        block_distances = np.sqrt(squared[np.arange(block.shape[0]), block_indices])
        indices[start : start + block.shape[0]] = block_indices
        distances[start : start + block.shape[0]] = block_distances
    return indices, distances


def rigid_transform_3d(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Fit the least-squares rigid SE(3) transform (Kabsch) source -> target."""
    source_array = np.asarray(source, dtype=np.float64).reshape(-1, 3)
    target_array = np.asarray(target, dtype=np.float64).reshape(-1, 3)
    if source_array.shape[0] < 3:
        raise ValueError("need at least 3 point pairs for a 3D rigid fit")
    source_center = source_array.mean(axis=0)
    target_center = target_array.mean(axis=0)
    covariance = (source_array - source_center).T @ (target_array - target_center)
    u_matrix, _, v_transpose = np.linalg.svd(covariance)
    rotation = v_transpose.T @ u_matrix.T
    if np.linalg.det(rotation) < 0.0:
        v_transpose[-1, :] *= -1.0
        rotation = v_transpose.T @ u_matrix.T
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = target_center - rotation @ source_center
    return transform


def iterative_closest_point(
    current_points: np.ndarray,
    previous_points: np.ndarray,
    initial_transform: np.ndarray,
    max_iterations: int = 25,
    max_correspondence_distance: float = 1.5,
    trim_fraction: float = 0.8,
    minimum_correspondences: int = 30,
    convergence_tolerance: float = 1e-4,
) -> Tuple[np.ndarray, float, int]:
    """Estimate the SE(3) transform taking the current cloud into the previous cloud."""
    current = np.asarray(current_points, dtype=np.float64).reshape(-1, 3)
    previous = np.asarray(previous_points, dtype=np.float64).reshape(-1, 3)
    if current.shape[0] < minimum_correspondences:
        raise ValueError("current cloud has too few valid points")
    if previous.shape[0] < minimum_correspondences:
        raise ValueError("previous cloud has too few valid points")
    transform = np.asarray(initial_transform, dtype=np.float64).reshape(4, 4).copy()
    previous_error = math.inf
    match_count = 0

    for _ in range(max(1, int(max_iterations))):
        transformed = transform_points(current, transform)
        indices, distances = nearest_neighbors(transformed, previous)
        accepted = np.flatnonzero(distances <= float(max_correspondence_distance))
        if accepted.size < minimum_correspondences:
            raise ValueError("cloud overlap is too small")
        keep_count = max(
            int(minimum_correspondences),
            int(accepted.size * min(1.0, max(0.1, float(trim_fraction)))),
        )
        order = np.argsort(distances[accepted])[:keep_count]
        source_indices = accepted[order]
        matched_source = transformed[source_indices]
        matched_target = previous[indices[source_indices]]
        correction = rigid_transform_3d(matched_source, matched_target)
        transform = correction @ transform
        error = float(np.sqrt(np.mean(np.square(distances[source_indices]))))
        match_count = int(source_indices.size)
        if abs(previous_error - error) < float(convergence_tolerance):
            return transform, error, match_count
        previous_error = error

    return transform, float(previous_error), int(match_count)


def rotation_angle(transform: np.ndarray) -> float:
    """Return the rotation magnitude [rad] of a homogeneous SE(3) transform."""
    rotation = np.asarray(transform, dtype=np.float64).reshape(4, 4)[:3, :3]
    trace = float(np.trace(rotation))
    cosine = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    return math.acos(cosine)


def matrix_to_quaternion(rotation: np.ndarray) -> Quaternion:
    """Convert a 3x3 rotation matrix to an (x, y, z, w) quaternion."""
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w_value = 0.25 * scale
        x_value = (matrix[2, 1] - matrix[1, 2]) / scale
        y_value = (matrix[0, 2] - matrix[2, 0]) / scale
        z_value = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w_value = (matrix[2, 1] - matrix[1, 2]) / scale
        x_value = 0.25 * scale
        y_value = (matrix[0, 1] + matrix[1, 0]) / scale
        z_value = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w_value = (matrix[0, 2] - matrix[2, 0]) / scale
        x_value = (matrix[0, 1] + matrix[1, 0]) / scale
        y_value = 0.25 * scale
        z_value = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w_value = (matrix[1, 0] - matrix[0, 1]) / scale
        x_value = (matrix[0, 2] + matrix[2, 0]) / scale
        y_value = (matrix[1, 2] + matrix[2, 1]) / scale
        z_value = 0.25 * scale
    norm_value = math.sqrt(
        x_value * x_value + y_value * y_value + z_value * z_value + w_value * w_value
    )
    if norm_value < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (
        x_value / norm_value,
        y_value / norm_value,
        z_value / norm_value,
        w_value / norm_value,
    )


def twist_from_delta(delta: np.ndarray, dt: float) -> Tuple[float, float, float, float, float, float]:
    """Express a current->previous ICP delta as body-frame linear/angular velocity."""
    matrix = np.asarray(delta, dtype=np.float64).reshape(4, 4)
    if not math.isfinite(dt) or dt <= 0.0:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    # Delta maps current-frame points into the previous frame, so the sensor
    # motion in the previous frame is +t. Body-frame velocity uses -R^T t.
    body_displacement = -rotation.T @ translation
    linear = body_displacement / dt
    angle = rotation_angle(matrix)
    if angle < 1e-9:
        angular = np.zeros(3, dtype=np.float64)
    else:
        axis = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ],
            dtype=np.float64,
        )
        sine = math.sqrt(max(0.0, axis.dot(axis))) * 0.5
        if sine < 1e-12:
            angular = np.zeros(3, dtype=np.float64)
        else:
            # ICP delta rotation is the inverse of the body motion; negate it.
            angular = -(axis / (2.0 * sine)) * (angle / dt)
    return (
        float(linear[0]),
        float(linear[1]),
        float(linear[2]),
        float(angular[0]),
        float(angular[1]),
        float(angular[2]),
    )


def points_from_xyz_iterable(points: Iterable[Vector3]) -> np.ndarray:
    """Pack an iterable of (x, y, z) tuples into an Nx3 array."""
    array = np.array([(float(p[0]), float(p[1]), float(p[2])) for p in points])
    if array.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    return np.asarray(array, dtype=np.float64).reshape(-1, 3)


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    """Compose two (x, y, z, w) quaternions as apply-b-then-a."""
    ax, ay, az, aw = (float(value) for value in a)
    bx, by, bz, bw = (float(value) for value in b)
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_conjugate(value: Quaternion) -> Quaternion:
    """Return the conjugate (inverse for unit quaternions) of a quaternion."""
    return (-float(value[0]), -float(value[1]), -float(value[2]), float(value[3]))


def quaternion_angle(value: Quaternion) -> float:
    """Return the rotation magnitude [rad] of an (x, y, z, w) quaternion."""
    w_value = max(-1.0, min(1.0, float(value[3])))
    return 2.0 * math.acos(abs(w_value))


def normalize_quaternion(value: Quaternion) -> Quaternion:
    """Normalize an (x, y, z, w) quaternion, falling back to identity."""
    norm_value = math.sqrt(sum(float(axis) * float(axis) for axis in value))
    if norm_value < 1e-12 or not math.isfinite(norm_value):
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(float(axis) / norm_value for axis in value)  # type: ignore[return-value]


def interpolate_samples(
    stamps: np.ndarray,
    positions: np.ndarray,
    quaternions: np.ndarray,
    stamp: float,
) -> Tuple[np.ndarray, Quaternion]:
    """Linearly interpolate buffered ground-truth samples at one timestamp.

    Positions use lerp; orientations use normalized lerp (nlerp), which is
    accurate for the small angles between 30 Hz ground-truth samples.
    Samples outside the buffered range clamp to the nearest end.
    """
    times = np.asarray(stamps, dtype=np.float64).reshape(-1)
    points = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
    quats = np.asarray(quaternions, dtype=np.float64).reshape(-1, 4)
    count = times.shape[0]
    if count == 0 or points.shape[0] != count or quats.shape[0] != count:
        raise ValueError("ground-truth buffer is empty or inconsistent")
    target = float(stamp)
    if target <= times[0]:
        return points[0].copy(), normalize_quaternion(tuple(quats[0].tolist()))
    if target >= times[-1]:
        return points[-1].copy(), normalize_quaternion(tuple(quats[-1].tolist()))
    upper = int(np.searchsorted(times, target, side="right"))
    lower = upper - 1
    span = times[upper] - times[lower]
    fraction = 0.0 if span <= 0.0 else (target - times[lower]) / span
    position = points[lower] * (1.0 - fraction) + points[upper] * fraction
    first = quats[lower].copy()
    second = quats[upper].copy()
    if float(np.dot(first, second)) < 0.0:
        second = -second
    blended = first * (1.0 - fraction) + second * fraction
    return (
        np.ascontiguousarray(position),
        normalize_quaternion(
            (float(blended[0]), float(blended[1]), float(blended[2]), float(blended[3]))
        ),
    )


def position_drift(
    estimated: np.ndarray,
    estimated_origin: np.ndarray,
    truth: np.ndarray,
    truth_origin: np.ndarray,
) -> Tuple[np.ndarray, float]:
    """Compare start-aligned displacements; odometry starts at its own origin.

    Returns the error vector (estimated displacement minus true displacement
    in the shared world frame) and its norm [m].
    """
    est_disp = np.asarray(estimated, dtype=np.float64).reshape(3) - np.asarray(
        estimated_origin, dtype=np.float64
    ).reshape(3)
    truth_disp = np.asarray(truth, dtype=np.float64).reshape(3) - np.asarray(
        truth_origin, dtype=np.float64
    ).reshape(3)
    error = est_disp - truth_disp
    return np.ascontiguousarray(error), float(np.linalg.norm(error))


def attitude_drift_angle(
    estimated: Quaternion,
    estimated_origin: Quaternion,
    truth: Quaternion,
    truth_origin: Quaternion,
) -> float:
    """Return the start-aligned attitude error [rad] between estimate and truth."""
    est_rel = quaternion_multiply(
        quaternion_conjugate(normalize_quaternion(estimated_origin)),
        normalize_quaternion(estimated),
    )
    truth_rel = quaternion_multiply(
        quaternion_conjugate(normalize_quaternion(truth_origin)),
        normalize_quaternion(truth),
    )
    mismatch = quaternion_multiply(quaternion_conjugate(truth_rel), est_rel)
    return quaternion_angle(mismatch)
