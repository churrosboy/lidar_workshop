import math
import warnings

import numpy as np

from gl5_localization import icp
from gl5_detection.detection_core import Point2D


# 배경 점 지도 학습, 매 스캔을 지도에 정렬해 지도 밖 점만 유지
class BackgroundModel:
    # 학습 프레임 수, 여유 거리, 정렬 허용치 설정
    def __init__(self, learn_frames=80, margin=0.15, ratio=0.02, min_fitness=0.5,
                 max_step=0.15, max_turn=math.radians(15), stride=3):
        if learn_frames < 1 or not all(math.isfinite(v) and v >= 0 for v in (margin, ratio)):
            raise ValueError('Invalid background parameters')
        self.learn_frames = int(learn_frames)
        self.margin, self.ratio = float(margin), float(ratio)
        self.min_fitness, self.max_step, self.max_turn = min_fitness, max_step, max_turn
        self.stride = max(1, int(stride))
        self.map_points = None
        self.target = None
        self.map_sector = (-math.pi, math.pi)
        self.pose = np.eye(3)
        self.fitness = 0.0
        self.aligned = False
        self.samples: list[np.ndarray] = []
        self.learning = False

    @property
    # 배경 지도 준비 여부
    def ready(self) -> bool:
        return self.map_points is not None

    # 학습 버퍼 비우기와 학습 시작
    def start_learning(self) -> None:
        self.samples = []
        self.learning = True

    # 지도, 포즈, 학습 상태 초기화
    def reset(self) -> None:
        self.map_points = self.target = None
        self.pose = np.eye(3)
        self.fitness, self.aligned = 0.0, False
        self.samples = []
        self.learning = False

    @staticmethod
    # 유효 빔의 (인덱스, xy 점) 배열
    def _points(ranges, angle_min, angle_increment):
        frame = np.asarray(ranges, dtype=float)
        index = np.nonzero(np.isfinite(frame) & (frame > 0))[0]
        angles = angle_min + index * angle_increment
        return index, np.column_stack((frame[index] * np.cos(angles), frame[index] * np.sin(angles)))

    # 학습 중 스캔 누적, 완료 시 빔별 중앙값으로 지도 생성
    def observe(self, ranges, angle_min, angle_increment) -> bool:
        if not self.learning:
            return False
        frame = np.asarray(ranges, dtype=float)
        frame = np.where(np.isfinite(frame) & (frame > 0), frame, np.nan)
        if self.samples and len(frame) != len(self.samples[0]):
            self.samples = []
        self.samples.append(frame)
        if len(self.samples) < self.learn_frames:
            return False
        stack = np.stack(self.samples)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            median = np.nanmedian(stack, axis=0)
        mostly_open = np.sum(np.isfinite(stack), axis=0) * 2 < len(stack)
        background = np.where(np.isnan(median) | mostly_open, np.inf, median)
        _, self.map_points = self._points(background, angle_min, angle_increment)
        self.target = icp.Target(self.map_points)
        self.map_sector = (angle_min, angle_min + (len(background) - 1) * angle_increment)
        self.pose, self.fitness, self.aligned = np.eye(3), 1.0, True
        self.samples = []
        self.learning = False
        return True

    # 스캔을 지도에 정렬 후 지도에서 떨어진 점만 유지, 나머지는 inf
    def foreground(self, ranges, angle_min, angle_increment) -> list[float]:
        if not self.ready:
            return list(ranges)
        index, points = self._points(ranges, angle_min, angle_increment)
        if len(points) == 0:
            return list(ranges)
        self._align(points)
        aligned = icp.apply(self.pose, points)
        frame = np.asarray(ranges, dtype=float)
        # ========================== Insert Your Code ==========================










        raise NotImplementedError('2단계 BackgroundModel.foreground')
        # ========================== Insert Your Code ==========================

    # ICP로 포즈 갱신, 실패하거나 튀면 이전 포즈 유지 (ICP 미구현이면 센서 고정 가정)
    def _align(self, points: np.ndarray) -> None:
        try:
            transform, fitness = icp.icp(points[::self.stride], self.target, self.pose)
        except NotImplementedError:
            self.fitness, self.aligned = 1.0, True
            return
        mx, my, myaw = icp.to_pose(np.linalg.inv(self.pose) @ transform)
        ok = (fitness >= self.min_fitness and math.hypot(mx, my) <= self.max_step
              and abs(myaw) <= self.max_turn)
        self.fitness = fitness
        if ok:
            self.pose = transform
        self.aligned = ok

    # 시각화용 지도 점 (현재 센서 좌표계)
    def contour(self, stride=5) -> list[Point2D]:
        if not self.ready:
            return []
        shown = icp.apply(np.linalg.inv(self.pose), self.map_points[::max(1, int(stride))])
        return [(float(x), float(y)) for x, y in shown]
