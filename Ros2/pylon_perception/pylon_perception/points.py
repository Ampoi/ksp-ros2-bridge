"""NumPy point-cloud primitives shared by PyLoN applications."""
import math
import numpy as np

def filter_range(
    points: np.ndarray,
    min_range: float,
    max_range: float,
) -> np.ndarray:
    """Drop non-finite points and points outside the range shell."""
    array = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if array.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float64)
    distances = np.linalg.norm(array, axis=1)
    valid = np.isfinite(distances)
    lower = max(0.001, float(min_range))
    upper = max(lower, float(max_range))
    valid &= distances >= lower
    valid &= distances <= upper
    return np.ascontiguousarray(array[valid])

def voxel_downsample(
    points: np.ndarray,
    voxel_size: float,
    max_points: int = 0,
) -> np.ndarray:
    """Average points per voxel, then deterministically stride-sample to a cap."""
    array = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    finite = np.all(np.isfinite(array), axis=1)
    array = array[finite]
    if array.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float64)
    voxel = float(voxel_size)
    if not math.isfinite(voxel) or voxel <= 0.0:
        downsampled = array
    else:
        keys = np.floor(array / voxel).astype(np.int64)
        sums: dict[tuple[int, int, int], np.ndarray] = {}
        counts: dict[tuple[int, int, int], int] = {}
        for key_tuple, point in zip(map(tuple, keys.tolist()), array):
            if key_tuple in sums:
                sums[key_tuple] += point
                counts[key_tuple] += 1
            else:
                sums[key_tuple] = point.copy()
                counts[key_tuple] = 1
        downsampled = np.array(
            [sums[key] / counts[key] for key in sums],
            dtype=np.float64,
        )
    if max_points > 0 and downsampled.shape[0] > max_points:
        indices = np.linspace(0, downsampled.shape[0] - 1, max_points, dtype=np.int64)
        downsampled = downsampled[indices]
    return np.ascontiguousarray(downsampled)

def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """Apply a homogeneous SE(3) transform to XYZ points."""
    array = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    matrix = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    rotated = array @ matrix[:3, :3].T + matrix[:3, 3]
    return np.ascontiguousarray(rotated)
