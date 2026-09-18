import math
import warnings

import numpy as np

warnings.filterwarnings('ignore', message='A NumPy version')
from scipy.spatial import cKDTree


# 유효 빔을 xy 점으로 바꾸고 stride마다 하나만 남긴다
def scan_to_points(ranges, angle_min, angle_increment, min_range=0.1, max_range=30.0,
                   stride=1) -> np.ndarray:
    distances = np.asarray(ranges, dtype=float)
    indices = np.arange(len(distances))
    valid = np.isfinite(distances) & (distances >= min_range) & (distances <= max_range)
    indices, distances = indices[valid][::max(1, int(stride))], distances[valid][::max(1, int(stride))]
    angles = angle_min + indices * angle_increment
    return np.column_stack((distances * np.cos(angles), distances * np.sin(angles)))


# (x, y, yaw) → 3x3 동차 변환
def make_transform(x: float, y: float, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


# 3x3 동차 변환 → (x, y, yaw)
def to_pose(transform: np.ndarray) -> tuple[float, float, float]:
    return (float(transform[0, 2]), float(transform[1, 2]),
            math.atan2(transform[1, 0], transform[0, 0]))


# 변환을 점 배열에 적용한다
def apply(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    return points @ transform[:2, :2].T + transform[:2, 2]


# 정합 대상 점군: KD 트리와 점별 직선 법선을 한 번만 계산해 둔다
class Target:
    # 점군에서 KD 트리와 법선을 만든다
    def __init__(self, points: np.ndarray, neighbours=8):
        self.points = np.asarray(points, dtype=float)
        self.tree = cKDTree(self.points)
        self.normals = self._normals(min(neighbours, len(self.points)))

    # 이웃 k개의 공분산에서 가장 작은 고유벡터를 법선으로 쓴다
    def _normals(self, k: int) -> np.ndarray:
        if k < 3:
            return np.zeros_like(self.points)
        _, index = self.tree.query(self.points, k=k)
        groups = self.points[index]
        centred = groups - groups.mean(axis=1, keepdims=True)
        covariance = np.einsum('nki,nkj->nij', centred, centred)
        _, vectors = np.linalg.eigh(covariance)
        return vectors[:, :, 0]


MAX_STEP_M = 1.0
MAX_STEP_RAD = math.radians(45)


# 점-대-선 ICP: src를 dst에 맞추는 변환과 대응 비율(fitness)을 돌려준다
def icp(src: np.ndarray, dst, init=None, iterations=20, max_dist=0.5, tolerance=1e-4,
        min_pairs=10) -> tuple[np.ndarray, float]:
    target = dst if isinstance(dst, Target) else Target(dst)
    initial = np.eye(3) if init is None else np.array(init, dtype=float)
    if len(src) < min_pairs or not np.all(np.isfinite(initial)):
        return initial, 0.0
    transform = initial
    current = apply(transform, src)

    # 현재 점들의 최근접 대응과 max_dist 이내 여부
    def matches():
        distances, neighbours = target.tree.query(current, distance_upper_bound=max_dist)
        matched = np.isfinite(distances)
        return matched, neighbours

    for _ in range(iterations):
        matched, neighbours = matches()
        if matched.sum() < min_pairs:
            break
        p = current[matched]
        q = target.points[neighbours[matched]]
        n = target.normals[neighbours[matched]]
        jacobian = np.column_stack((n[:, 0], n[:, 1], n[:, 0] * -p[:, 1] + n[:, 1] * p[:, 0]))
        residual = np.einsum('ij,ij->i', n, q - p)
        (dx, dy, dtheta), *_ = np.linalg.lstsq(jacobian, residual, rcond=1e-6)
        if (not all(map(math.isfinite, (dx, dy, dtheta))) or math.hypot(dx, dy) > MAX_STEP_M
                or abs(dtheta) > MAX_STEP_RAD):
            return initial, 0.0
        step = make_transform(dx, dy, dtheta)
        transform = step @ transform
        current = apply(step, current)
        if math.hypot(dx, dy) < tolerance and abs(dtheta) < tolerance:
            break
    matched, _ = matches()
    return transform, float(matched.mean())
