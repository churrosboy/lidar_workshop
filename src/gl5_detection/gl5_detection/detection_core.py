"""Obstacle detection algorithms, independent of ROS, RViz and file storage.

cluster_scan(): turn laser ranges into clusters inside the selected polygon.
BoxTracker: associate cluster boxes across scans and estimate relative speed.
Occupancy: debounce detection into an occupied/clear decision.
"""
from collections import deque
from dataclasses import dataclass, field, replace
import math


# Geometry, scan clustering and occupancy

Point2D = tuple[float, float]
BoundingBox = tuple[float, float, float, float]

EPS = 1e-9


def cross(a: Point2D, b: Point2D, p: Point2D) -> float:
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])


def on_segment(a: Point2D, b: Point2D, p: Point2D) -> bool:
    return abs(cross(a, b, p)) <= EPS and all(
        min(a[i], b[i]) - EPS <= p[i] <= max(a[i], b[i]) + EPS for i in (0, 1))


def intersects(a: Point2D, b: Point2D, c: Point2D, d: Point2D) -> bool:
    return (cross(a, b, c) * cross(a, b, d) < 0 and
            cross(c, d, a) * cross(c, d, b) < 0) or any((
        on_segment(a, b, c), on_segment(a, b, d),
        on_segment(c, d, a), on_segment(c, d, b)))


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


def inside(point: Point2D, polygon: list[Point2D]) -> bool:
    """Ray casting; includes the boundary, supports concave polygons."""
    result = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if on_segment(a, b, point):
            return True
        if (a[1] > point[1]) != (b[1] > point[1]):
            x = a[0] + (point[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            if point[0] < x:
                result = not result
    return result


def cluster_scan(ranges, angle_min, angle_increment, range_min, range_max, polygon,
             min_points=5, max_gap=0.15) -> list[list[Point2D]]:
    """Cluster nearby in-ROI returns, skipping invalid beams; outside beams break it."""
    groups, current = [], []
    def flush():
        if len(current) >= min_points:
            groups.append(current.copy())
        current.clear()
    for i, distance in enumerate(ranges):
        if not math.isfinite(distance) or distance <= 0 or not range_min <= distance <= range_max:
            continue
        angle = angle_min + i * angle_increment
        point = (distance * math.cos(angle), distance * math.sin(angle))
        if not inside(point, polygon):
            flush()
            continue
        if current and math.dist(current[-1], point) > max_gap:
            flush()
        current.append(point)
    flush()
    return groups


def bounds(points: list[Point2D]) -> BoundingBox:
    return (min(p[0] for p in points), min(p[1] for p in points),
            max(p[0] for p in points), max(p[1] for p in points))


class Occupancy:
    """Require a continuous detection/clear interval before changing occupancy."""

    def __init__(self, enter=0.2, leave=0.5):
        self.enter, self.leave = enter, leave
        self.reset()

    def reset(self):
        self.occupied = False
        self.candidate = None
        self.since = None

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


# Obstacle association and sensor-relative speed

# A longer observation gap invalidates the old velocity estimate.
SPEED_RESET_GAP = 0.2
MIN_SPEED_SAMPLES = 3
MIN_SPEED_DURATION = 0.15


@dataclass
class Track:
    id: int
    center: Point2D
    box: BoundingBox
    last_seen: float
    history: deque[tuple[float, float, float]] = field(default_factory=deque)
    velocity: Point2D = (0.0, 0.0)
    speed: float | None = None


class BoxTracker:
    def __init__(self, match_distance=0.4, max_age=0.5, window=0.4):
        self.match_distance = match_distance
        self.max_age = max_age
        self.window = window
        self.next_id = 1
        self.tracks: dict[int, Track] = {}

    def reset(self) -> None:
        # IDs remain monotonic across resets, as they do across expired tracks.
        self.tracks.clear()

    def update(self, groups: list[list[Point2D]], now: float) -> list[Track]:
        self._expire_tracks(now)
        boxes = [bounds(group) for group in groups]
        centers = [((x0 + x1) / 2, (y0 + y1) / 2) for x0, y0, x1, y1 in boxes]
        assignments = self._match_tracks(centers, now)

        visible = []
        for index, center in enumerate(centers):
            if index not in assignments:
                track = Track(self.next_id, center, boxes[index], now)
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
            # Copy the observation so later updates do not change its box/speed.
            visible.append(replace(track))
        return visible

    def _expire_tracks(self, now: float) -> None:
        self.tracks = {
            key: track for key, track in self.tracks.items()
            if 0 <= now - track.last_seen <= self.max_age
        }

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
        # Keep distance, track ID and detection index as the original tie breakers.
        for _, key, index in sorted(candidates):
            if key not in used_tracks and index not in assignments:
                used_tracks.add(key)
                assignments[index] = key
        return assignments

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

        # Regression over a short window reduces frame-to-frame range noise.
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
