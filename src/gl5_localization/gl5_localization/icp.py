"""2D point-to-point ICP on laser scans, independent of ROS.

scan_to_points(): LaserScan ranges -> (N, 2) array in the sensor frame.
Target: a reference scan with its KD-tree and line normals, built once per keyframe.
icp(): point-to-line ICP estimating the rigid transform T that maps src onto the Target.
Transforms are 3x3 homogeneous matrices; make_transform()/to_pose() convert (x, y, yaw).
"""
import math
import warnings

import numpy as np

warnings.filterwarnings('ignore', message='A NumPy version')  # scipy 1.8 vs numpy 1.26
from scipy.spatial import cKDTree  # noqa: E402


def scan_to_points(ranges, angle_min, angle_increment, min_range=0.1, max_range=30.0,
                   stride=1) -> np.ndarray:
    """Valid returns as x/y points; every stride-th valid beam is kept."""
    distances = np.asarray(ranges, dtype=float)
    indices = np.arange(len(distances))
    valid = np.isfinite(distances) & (distances >= min_range) & (distances <= max_range)
    indices, distances = indices[valid][::max(1, int(stride))], distances[valid][::max(1, int(stride))]
    angles = angle_min + indices * angle_increment
    return np.column_stack((distances * np.cos(angles), distances * np.sin(angles)))


def make_transform(x: float, y: float, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def to_pose(transform: np.ndarray) -> tuple[float, float, float]:
    return (float(transform[0, 2]), float(transform[1, 2]),
            math.atan2(transform[1, 0], transform[0, 0]))


def apply(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    return points @ transform[:2, :2].T + transform[:2, 2]


class Target:
    """A reference point set with its KD-tree and per-point line normals (computed once)."""

    def __init__(self, points: np.ndarray, neighbours=8):
        self.points = np.asarray(points, dtype=float)
        self.tree = cKDTree(self.points)
        self.normals = self._normals(min(neighbours, len(self.points)))

    def _normals(self, k: int) -> np.ndarray:
        if k < 3:
            return np.zeros_like(self.points)
        _, index = self.tree.query(self.points, k=k)
        groups = self.points[index]                      # (N, k, 2)
        centred = groups - groups.mean(axis=1, keepdims=True)
        covariance = np.einsum('nki,nkj->nij', centred, centred)
        _, vectors = np.linalg.eigh(covariance)          # ascending eigenvalues
        return vectors[:, :, 0]                          # direction of least spread


def icp(src: np.ndarray, dst, init=None, iterations=20, max_dist=0.5, tolerance=1e-4,
        min_pairs=10) -> tuple[np.ndarray, float]:
    """Point-to-line ICP: align src to dst. Returns (T with apply(T, src) ~ dst, fitness).

    dst is an (N, 2) array or a prebuilt Target. Each iteration matches every src point
    to its nearest dst point within max_dist and solves the linearised least-squares
    problem for (dx, dy, dtheta) that minimises the distance along the dst normals, so
    points may slide along walls without penalty. fitness is the fraction of src points
    matched on the last iteration; treat a low value as a failed match. init is the
    starting guess (e.g. the previous motion).
    """
    target = dst if isinstance(dst, Target) else Target(dst)
    transform = np.eye(3) if init is None else np.array(init, dtype=float)
    current = apply(transform, src)
    fitness = 0.0
    if len(src) < min_pairs:
        return transform, fitness
    for _ in range(iterations):
        distances, neighbours = target.tree.query(current, distance_upper_bound=max_dist)
        matched = np.isfinite(distances)
        fitness = float(matched.mean())
        if matched.sum() < min_pairs:
            break
        p = current[matched]
        q = target.points[neighbours[matched]]
        n = target.normals[neighbours[matched]]
        # Residual n.(R p + t - q) with R linearised: R p ~ p + theta * (-p_y, p_x).
        jacobian = np.column_stack((n[:, 0], n[:, 1], n[:, 0] * -p[:, 1] + n[:, 1] * p[:, 0]))
        residual = np.einsum('ij,ij->i', n, q - p)
        (dx, dy, dtheta), *_ = np.linalg.lstsq(jacobian, residual, rcond=None)
        step = make_transform(dx, dy, dtheta)
        transform = step @ transform
        current = apply(step, current)
        if math.hypot(dx, dy) < tolerance and abs(dtheta) < tolerance:
            break
    return transform, fitness
