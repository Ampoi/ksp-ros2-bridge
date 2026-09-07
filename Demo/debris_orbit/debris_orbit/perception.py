"""SciPy point-cloud clustering and a relative-motion Kalman filter."""
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.linalg import solve


@dataclass
class Cluster:
    points: np.ndarray
    center: np.ndarray


def extract_clusters(points, voxel_size=.15, tolerance=1.5, min_points=5,
                     min_range=1., max_range=250., max_points=4000, max_off_axis_deg=35., max_extent=10.):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    ranges = np.linalg.norm(points, axis=1)
    points = points[np.isfinite(points).all(axis=1) & (ranges >= min_range) & (ranges <= max_range)
                    & (points[:, 0] >= ranges * np.cos(np.deg2rad(max_off_axis_deg)))]
    if not len(points):
        return []
    _, indices = np.unique(np.floor(points / voxel_size), axis=0, return_index=True)
    points = points[np.sort(indices)]
    if len(points) > max_points:
        points = points[np.linspace(0, len(points)-1, max_points, dtype=int)]
    pairs = cKDTree(points).query_pairs(tolerance, output_type='ndarray')
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(len(points), len(points)))
    count, labels = connected_components(graph, directed=False)
    clusters = []
    for label in range(count):
        subset = points[labels == label]
        if len(subset) >= min_points and np.linalg.norm(np.ptp(subset, axis=0)) <= max_extent:
            clusters.append(Cluster(subset, (subset.min(axis=0) + subset.max(axis=0)) / 2))
    return sorted(clusters, key=lambda cluster: len(cluster.points), reverse=True)


class RelativeTracker:
    """Constant-velocity filter in fixed world axes, entirely relative to the chaser."""
    def __init__(self, measurement_sigma=.25, acceleration_sigma=.5, max_jump=3., max_speed=5., timeout=1.):
        self.measurement_sigma = measurement_sigma
        self.acceleration_sigma = acceleration_sigma
        self.max_jump, self.max_speed, self.timeout = max_jump, max_speed, timeout
        self.reset()

    def reset(self):
        self.state = None
        self.covariance = None
        self.stamp = None
        self.observations = 0

    def update(self, candidates, stamp, initial_index=0):
        candidates = np.asarray(candidates, dtype=float).reshape(-1, 3)
        if not len(candidates) or not np.isfinite(candidates).all() or not np.isfinite(stamp):
            return None
        if self.stamp is not None and stamp <= self.stamp:
            return None
        if self.stamp is not None and stamp - self.stamp > self.timeout:
            self.reset()
        if self.state is None:
            if initial_index >= len(candidates):
                return None
            index = initial_index
            self.state = np.r_[candidates[index], np.zeros(3)]
            self.covariance = np.diag([self.measurement_sigma**2]*3 + [1.]*3)
        else:
            dt = stamp - self.stamp
            transition = np.eye(6); transition[:3, 3:] = np.eye(3) * dt
            noise_map = np.vstack([np.eye(3) * dt**2 / 2, np.eye(3) * dt])
            prediction = transition @ self.state
            covariance = transition @ self.covariance @ transition.T + noise_map @ noise_map.T * self.acceleration_sigma**2
            distances = np.linalg.norm(candidates - prediction[:3], axis=1)
            index = int(np.argmin(distances))
            if distances[index] > self.max_jump:
                return None
            innovation_cov = covariance[:3, :3] + np.eye(3) * self.measurement_sigma**2
            gain = solve(innovation_cov, covariance[:3, :], assume_a='pos').T
            self.state = prediction + gain @ (candidates[index] - prediction[:3])
            correction = np.eye(6); correction[:, :3] -= gain
            # Joseph form keeps the covariance positive semidefinite.
            self.covariance = correction @ covariance @ correction.T + gain @ gain.T * self.measurement_sigma**2
            speed = np.linalg.norm(self.state[3:])
            if speed > self.max_speed:
                self.state[3:] *= self.max_speed / speed
        self.stamp = stamp
        self.observations += 1
        return index, self.state[:3].copy(), self.state[3:].copy()
