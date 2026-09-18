import math
import warnings

import numpy as np

warnings.filterwarnings('ignore', message='A NumPy version')
from scipy.spatial import cKDTree


# 유효 빔 → xy 점 변환, stride마다 하나만 유지
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


# 점 배열에 변환 적용
def apply(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    return points @ transform[:2, :2].T + transform[:2, 2]


# 정합 대상 점군: KD 트리와 점별 직선 법선을 한 번만 계산
class Target:
    # 점군의 KD 트리와 법선 생성
    def __init__(self, points: np.ndarray, neighbours=8):
        self.points = np.asarray(points, dtype=float)
        self.tree = cKDTree(self.points)
        self.normals = self._normals(min(neighbours, len(self.points)))

    # 이웃 k개 공분산의 최소 고유벡터를 법선으로 사용
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


# 점-대-선 ICP: src를 dst에 맞추는 변환과 대응 비율(fitness) 반환
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
        # ===== [4단계 실습] 점-대-선 ICP 한 반복을 채우세요 =====
        # 주어진 것: p = 현재 스캔 점 (M, 2), q = 대응되는 지도 점 (M, 2), n = q에서의 직선 법선 (M, 2)
        # 목표: p를 (dx, dy)만큼 옮기고 dtheta만큼 돌렸을 때 n·(p' - q) 가 최소가 되는 (dx, dy, dtheta)
        # 회전을 작다고 보고 선형화하면 R p ≈ p + dtheta * (-p_y, p_x) 이므로 잔차는
        #   n·(p - q) + n_x*dx + n_y*dy + dtheta*(n_x*(-p_y) + n_y*p_x)
        # 순서:
        #  1. jacobian = np.column_stack((n_x, n_y, n_x*(-p_y) + n_y*p_x))   # (M, 3)
        #  2. residual = n·(q - p) 를 점마다 계산 (np.einsum('ij,ij->i', n, q - p))
        #  3. (dx, dy, dtheta), *_ = np.linalg.lstsq(jacobian, residual, rcond=1e-6)
        #  4. 발산 검사: 값이 유한하지 않거나 hypot(dx, dy) > MAX_STEP_M 또는 |dtheta| > MAX_STEP_RAD
        #     이면 return initial, 0.0
        #  5. step = make_transform(dx, dy, dtheta); transform = step @ transform; current = apply(step, current)
        #  6. hypot(dx, dy) < tolerance 이고 |dtheta| < tolerance 이면 break (수렴)
        # 채점: pytest test/test_icp.py   (gl5_localization 폴더에서)
        raise NotImplementedError('4단계: icp 반복 스텝을 채우세요')
    matched, _ = matches()
    return transform, float(matched.mean())
