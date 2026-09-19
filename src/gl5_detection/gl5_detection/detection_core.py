from collections import deque
from dataclasses import dataclass, field, replace
import math

import numpy as np


Point2D = tuple[float, float]
BoundingBox = tuple[float, float, float, float]

EPS = 1e-9


# 벡터 ab와 ap의 외적 (부호로 좌우 판정)
def cross(a: Point2D, b: Point2D, p: Point2D) -> float:
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])


# 점 p가 선분 ab 위에 있는지
def on_segment(a: Point2D, b: Point2D, p: Point2D) -> bool:
    return abs(cross(a, b, p)) <= EPS and all(
        min(a[i], b[i]) - EPS <= p[i] <= max(a[i], b[i]) + EPS for i in (0, 1))


# 선분 ab와 cd가 만나는지
def intersects(a: Point2D, b: Point2D, c: Point2D, d: Point2D) -> bool:
    return (cross(a, b, c) * cross(a, b, d) < 0 and
            cross(c, d, a) * cross(c, d, b) < 0) or any((
        on_segment(a, b, c), on_segment(a, b, d),
        on_segment(c, d, a), on_segment(c, d, b)))


# 꼭짓점 개수, 유한성, 중복, 자기 교차, 면적 검사 후 폴리곤 확정
def validate_polygon(vertices) -> list[Point2D]:
    if not isinstance(vertices, (list, tuple)) or not 3 <= len(vertices) <= 100:
        raise ValueError('Use 3 to 100 vertices')
    if any(not isinstance(p, (list, tuple)) or len(p) != 2 or
           any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for v in p) for p in vertices):
        raise ValueError('Vertices must be finite [x, y] coordinates')
    n = len(vertices)
    if any(math.dist(vertices[i], vertices[j]) < 1e-6 for i in range(n) for j in range(i)):
        raise ValueError('Duplicate vertices')
    for i in range(n):
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue
            if intersects(vertices[i], vertices[(i + 1) % n], vertices[j], vertices[(j + 1) % n]):
                raise ValueError('Polygon edges cross or touch')
    area = abs(sum(vertices[i][0] * vertices[(i + 1) % n][1] -
                   vertices[(i + 1) % n][0] * vertices[i][1] for i in range(n))) / 2
    if area < 1e-4:
        raise ValueError('Polygon area is too small')
    return [(float(p[0]), float(p[1])) for p in vertices]


# 점이 폴리곤 안(경계 포함)에 있는지 (레이 캐스팅)
def inside(point: Point2D, polygon: list[Point2D]) -> bool:
    result = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if on_segment(a, b, point):
            return True
        if (a[1] > point[1]) != (b[1] > point[1]):
            x = a[0] + (point[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            if point[0] < x:
                result = not result
    return result


# (x, y) 목록에 3x3 변환 적용
def transform_points(transform, points) -> list[Point2D]:
    if not points:
        return []
    transform = np.asarray(transform, dtype=float)
    moved = np.asarray(points, dtype=float) @ transform[:2, :2].T + transform[:2, 2]
    return [(float(x), float(y)) for x, y in moved]


# 이웃 유효점 군집화 (polygon이 None이면 전체 시야)
def cluster_scan(ranges, angle_min, angle_increment, range_min, range_max, polygon,
             min_points=5, max_gap=0.15) -> list[list[Point2D]]:
    groups, current = [], []
    # 현재 묶음이 최소 점 수를 넘으면 군집 확정
    def flush():
        if len(current) >= min_points:
            groups.append(current.copy())
        current.clear()
    for i, distance in enumerate(ranges):
        # 무효한 빔은 건너뜁니다 (거리가 inf/NaN 이거나 센서 유효 범위 밖).
        if not math.isfinite(distance) or distance <= 0 or not range_min <= distance <= range_max:
            continue
        # 극좌표(거리, 각도) -> 센서 기준 직교좌표 (x, y)
        angle = angle_min + i * angle_increment
        point = (distance * math.cos(angle), distance * math.sin(angle))
        # ========================== Insert Your Code ==========================
        # 위에서 만든 point 를 가지고 아래 세 가지를 처리하세요.
        #
        #   1) polygon 이 주어졌는데(None 이 아닌데) point 가 그 밖이라면
        #      지금까지 모은 묶음을 flush() 로 끊고 이 점은 버립니다 (continue).
        #      폴리곤 안/밖 판정은 이 파일의 inside(point, polygon) 을 씁니다.
        #
        #   2) current 에 앞점이 있고, 그 앞점과 point 사이 거리가 max_gap 보다 멀다면
        #      다른 물체가 시작된 것이므로 flush() 로 끊습니다.
        #      단 이때 point 는 버리지 않습니다. 두 점 사이 거리는 math.dist(a, b) 입니다.
        #
        #   3) point 를 current 에 추가합니다.

        raise NotImplementedError('1단계 cluster_scan')
        # ========================== Insert Your Code ==========================
    flush()
    return groups


# 점들의 바운딩 박스 (x0, y0, x1, y1)
def bounds(points: list[Point2D]) -> BoundingBox:
    return (min(p[0] for p in points), min(p[1] for p in points),
            max(p[0] for p in points), max(p[1] for p in points))


# 감지/미감지가 일정 시간 지속돼야 점유 상태를 바꾸는 디바운스
class Occupancy:
    # 진입/이탈 지연 시간 설정
    def __init__(self, enter=0.2, leave=0.5):
        self.enter, self.leave = enter, leave
        self.reset()

    # 점유 상태와 후보 초기화
    def reset(self):
        self.occupied = False
        self.candidate = None
        self.since = None

    # 이번 프레임 감지 여부로 점유 상태 갱신
    def update(self, detected, now):
        if detected == self.occupied:
            self.candidate = self.since = None
        else:
            if self.candidate != detected:
                self.candidate, self.since = detected, now
            if now - self.since >= (self.enter if detected else self.leave):
                self.occupied = detected
                self.candidate = self.since = None
        return self.occupied


SPEED_RESET_GAP = 0.2
MIN_SPEED_SAMPLES = 3
MIN_SPEED_DURATION = 0.15


@dataclass
# 추적 중인 물체 하나 (박스, 속도, 궤적, 영역 관계)
class Track:
    id: int
    center: Point2D
    box: BoundingBox
    last_seen: float
    history: deque[tuple[float, float, float]] = field(default_factory=deque)
    velocity: Point2D = (0.0, 0.0)
    speed: float | None = None
    trail: deque[Point2D] = field(default_factory=deque)
    in_region: bool = False
    time_to_enter: float | None = None
    entry_point: Point2D | None = None


# 군집 박스의 프레임 간 연결과 속도 추정
class BoxTracker:
    # 연결 거리, 유지 시간, 속도 창, 궤적 길이 설정
    def __init__(self, match_distance=0.4, max_age=0.5, window=0.4, trail_length=60):
        self.match_distance = match_distance
        self.max_age = max_age
        self.window = window
        self.trail_length = int(trail_length)
        self.next_id = 1
        self.tracks: dict[int, Track] = {}

    # 모든 추적 삭제
    def reset(self) -> None:
        self.tracks.clear()

    # 군집을 기존 추적에 연결 또는 새 추적 생성, 스냅샷 반환
    def update(self, groups: list[list[Point2D]], now: float) -> list[Track]:
        self._expire_tracks(now)
        boxes = [bounds(group) for group in groups]
        centers = [((x0 + x1) / 2, (y0 + y1) / 2) for x0, y0, x1, y1 in boxes]
        assignments = self._match_tracks(centers, now)

        visible = []
        for index, center in enumerate(centers):
            if index not in assignments:
                track = Track(self.next_id, center, boxes[index], now,
                              trail=deque(maxlen=self.trail_length))
                self.next_id += 1
                self.tracks[track.id] = track
            else:
                track = self.tracks[assignments[index]]
                if now - track.last_seen > SPEED_RESET_GAP:
                    track.history.clear()
                    track.velocity = (0.0, 0.0)

            self._update_speed(track, center, now)
            track.center = center
            track.last_seen = now
            track.box = boxes[index]
            track.trail.append(center)
            visible.append(replace(track, trail=deque(track.trail, maxlen=self.trail_length)))
        return visible

    # 오래 안 보인 추적 제거
    def _expire_tracks(self, now: float) -> None:
        self.tracks = {
            key: track for key, track in self.tracks.items()
            if 0 <= now - track.last_seen <= self.max_age
        }

    # 예측 위치와 가까운 군집을 추적에 1:1 배정
    def _match_tracks(self, centers: list[Point2D], now: float) -> dict[int, int]:
        candidates = []
        for key, track in self.tracks.items():
            elapsed = now - track.last_seen
            predicted = tuple(
                track.center[axis] + track.velocity[axis] * elapsed for axis in (0, 1)
            )
            for index, center in enumerate(centers):
                distance = math.dist(predicted, center)
                if distance <= self.match_distance:
                    candidates.append((distance, key, index))

        used_tracks = set()
        assignments = {}
        for _, key, index in sorted(candidates):
            if key not in used_tracks and index not in assignments:
                used_tracks.add(key)
                assignments[index] = key
        return assignments

    # 최근 위치 이력의 직선 회귀로 속도 계산
    def _update_speed(self, track: Track, center: Point2D, now: float) -> None:
        history = track.history
        if not history or now > history[-1][0]:
            history.append((now, *center))
        while len(history) > 1 and now - history[0][0] > self.window:
            history.popleft()

        track.speed = None
        if not (len(history) >= MIN_SPEED_SAMPLES
                and history[-1][0] - history[0][0] >= MIN_SPEED_DURATION):
            return

        times = [sample[0] - history[0][0] for sample in history]
        mean_time = sum(times) / len(times)
        denominator = sum((time - mean_time) ** 2 for time in times)
        velocities = []
        for axis in (1, 2):
            mean_position = sum(sample[axis] for sample in history) / len(history)
            velocities.append(sum(
                (time - mean_time) * (sample[axis] - mean_position)
                for time, sample in zip(times, history)
            ) / denominator)
        track.velocity = tuple(velocities)
        track.speed = math.hypot(*velocities)


# 각 추적의 영역 안 여부만 채우고 예측 정보는 비움 (예측 기능 꺼짐/미구현 시 사용)
def mark_in_region(tracks: list[Track], polygon: list[Point2D]) -> list[Track]:
    for track in tracks:
        track.in_region = inside(track.center, polygon)
        track.time_to_enter, track.entry_point = None, None
    return tracks
