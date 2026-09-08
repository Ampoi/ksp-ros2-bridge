"""Pure NumPy scan conversion and planar ICP routines."""

import math
from typing import Iterable, Tuple

import numpy as np


def laser_points(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    max_points: int,
) -> np.ndarray:
    """Convert valid LaserScan ranges into a bounded planar point array."""
    ranges_array = np.asarray(tuple(ranges), dtype=np.float64)
    if ranges_array.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    angles = angle_min + np.arange(ranges_array.size) * angle_increment
    valid = np.isfinite(ranges_array)
    valid &= ranges_array >= max(0.001, range_min)
    valid &= ranges_array <= range_max
    ranges_array = ranges_array[valid]
    angles = angles[valid]
    points = np.column_stack(
        (ranges_array * np.cos(angles), ranges_array * np.sin(angles))
    )
    if max_points > 0 and points.shape[0] > max_points:
        indices = np.linspace(0, points.shape[0] - 1, max_points, dtype=np.int64)
        points = points[indices]
    return points


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """Apply a homogeneous SE(2) transform to planar points."""
    return points @ transform[:2, :2].T + transform[:2, 2]


def nearest_neighbors(source: np.ndarray, target: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Find a target neighbor and distance for each source point."""
    differences = source[:, np.newaxis, :] - target[np.newaxis, :, :]
    squared_distances = np.einsum("ijk,ijk->ij", differences, differences)
    indices = np.argmin(squared_distances, axis=1)
    distances = np.sqrt(squared_distances[np.arange(source.shape[0]), indices])
    return indices, distances


def rigid_transform(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Fit the least-squares rigid SE(2) transform between paired points."""
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    centered_source = source - source_center
    centered_target = target - target_center
    covariance = centered_source.T @ centered_target
    u_matrix, _, v_transpose = np.linalg.svd(covariance)
    rotation = v_transpose.T @ u_matrix.T
    if np.linalg.det(rotation) < 0.0:
        v_transpose[-1, :] *= -1.0
        rotation = v_transpose.T @ u_matrix.T
    translation = target_center - rotation @ source_center
    transform = np.eye(3, dtype=np.float64)
    transform[:2, :2] = rotation
    transform[:2, 2] = translation
    return transform


def iterative_closest_point(
    current_points: np.ndarray,
    previous_points: np.ndarray,
    initial_transform: np.ndarray,
    max_iterations: int = 20,
    max_correspondence_distance: float = 1.0,
    trim_fraction: float = 0.8,
    minimum_correspondences: int = 20,
    convergence_tolerance: float = 1e-4,
) -> Tuple[np.ndarray, float, int]:
    """Estimate the SE(2) transform from the current scan into the previous scan."""
    if current_points.shape[0] < minimum_correspondences:
        raise ValueError("current scan has too few valid points")
    if previous_points.shape[0] < minimum_correspondences:
        raise ValueError("previous scan has too few valid points")
    transform = np.asarray(initial_transform, dtype=np.float64).copy()
    previous_error = math.inf
    match_count = 0

    for _ in range(max_iterations):
        transformed = transform_points(current_points, transform)
        indices, distances = nearest_neighbors(transformed, previous_points)
        accepted_indices = np.flatnonzero(distances <= max_correspondence_distance)
        if accepted_indices.size < minimum_correspondences:
            raise ValueError("scan overlap is too small")

        keep_count = max(
            minimum_correspondences,
            int(accepted_indices.size * min(1.0, max(0.1, trim_fraction))),
        )
        order = np.argsort(distances[accepted_indices])[:keep_count]
        source_indices = accepted_indices[order]
        matched_source = transformed[source_indices]
        matched_target = previous_points[indices[source_indices]]
        correction = rigid_transform(matched_source, matched_target)
        transform = correction @ transform
        error = float(np.sqrt(np.mean(np.square(distances[source_indices]))))
        match_count = source_indices.size
        if abs(previous_error - error) < convergence_tolerance:
            return transform, error, match_count
        previous_error = error

    return transform, previous_error, match_count


def yaw_from_transform(transform: np.ndarray) -> float:
    """Extract yaw from a homogeneous SE(2) transform."""
    return math.atan2(transform[1, 0], transform[0, 0])
