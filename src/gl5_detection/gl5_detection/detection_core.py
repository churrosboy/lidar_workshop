"""Obstacle detection algorithms, independent of ROS, RViz and file storage.

BackgroundModel: learn the free-space distance per beam and keep only closer returns.
cluster_scan(): turn laser ranges into clusters (inside a polygon, or the whole scan).
BoxTracker: associate cluster boxes across scans, keep a trail and estimate speed.
predict_entry()/classify_tracks(): extrapolate tracks and find when they enter the region.
Occupancy: debounce detection into an occupied/clear decision.
"""
from collections import deque
from dataclasses import dataclass, field, replace
import math
import warnings

import numpy as np


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


class BackgroundModel:
    """Per-beam free-space distance learned from a burst of scans.

    While learning, observe() accumulates frames; the per-beam median makes a person
    walking through during learning harmless. Afterwards foreground() keeps only beams
    that return clearly closer than the background (margin + ratio * distance).
    A beam whose background is inf (no return, open space) is foreground whenever it
    returns anything. If the beam count changes (different sensor), learning restarts.
    """

    def __init__(self, learn_frames=80, margin=0.15, ratio=0.02):
        if learn_frames < 1 or not all(math.isfinite(v) and v >= 0 for v in (margin, ratio)):
            raise ValueError('Invalid background parameters')
        self.learn_frames = int(learn_frames)
        self.margin = float(margin)
        self.ratio = float(ratio)
        self.background = None
        self.samples: list[np.ndarray] = []
        self.learning = False

    @property
    def ready(self) -> bool:
        return self.background is not None

    def start_learning(self) -> None:
        self.samples = []
        self.learning = True

    def reset(self) -> None:
        self.background = None
        self.samples = []
        self.learning = False

    @staticmethod
    def _frame(ranges) -> np.ndarray:
        frame = np.asarray(ranges, dtype=float)
        return np.where(np.isfinite(frame) & (frame > 0), frame, np.nan)

    def observe(self, ranges) -> bool:
        """Accumulate one scan while learning; True when the model just became ready."""
        if not self.learning:
            return False
        frame = self._frame(ranges)
        if self.samples and len(frame) != len(self.samples[0]):
            self.samples = []
        self.samples.append(frame)
        if len(self.samples) < self.learn_frames:
            return False
        stack = np.stack(self.samples)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)  # all-NaN beams are open space
            median = np.nanmedian(stack, axis=0)
        # A beam without a return in most frames is open space, even if something
        # passed through it briefly while learning.
        mostly_open = np.sum(np.isfinite(stack), axis=0) * 2 < len(stack)
        self.background = np.where(np.isnan(median) | mostly_open, np.inf, median)
        self.samples = []
        self.learning = False
        return True

    def foreground(self, ranges) -> list[float]:
        """Return ranges with background beams set to inf; passthrough until ready."""
        if self.background is None:
            return list(ranges)
        frame = np.asarray(ranges, dtype=float)
        if len(frame) != len(self.background):
            self.reset()
            self.start_learning()
            return list(ranges)
        finite = np.isfinite(self.background)
        threshold = np.full_like(self.background, np.inf)
        threshold[finite] = self.background[finite] * (1.0 - self.ratio) - self.margin
        keep = np.isfinite(frame) & (frame > 0) & (frame < threshold)
        return np.where(keep, frame, np.inf).tolist()

    def contour(self, angle_min, angle_increment, stride=5) -> list[Point2D]:
        """Sampled background outline in sensor coordinates, for visualization."""
        if self.background is None:
            return []
        points = []
        for index in range(0, len(self.background), max(1, int(stride))):
            distance = self.background[index]
            if not math.isfinite(distance):
                continue
            angle = angle_min + index * angle_increment
            points.append((float(distance * math.cos(angle)), float(distance * math.sin(angle))))
        return points


def cluster_scan(ranges, angle_min, angle_increment, range_min, range_max, polygon,
             min_points=5, max_gap=0.15) -> list[list[Point2D]]:
    """Cluster nearby returns, skipping invalid beams.

    With a polygon only in-ROI returns are kept and outside beams break a cluster;
    with polygon=None the whole scan is clustered (region checks happen per track).
    """
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
        if polygon is not None and not inside(point, polygon):
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
    # Past centers for drawing; length is BoxTracker(trail_length).
    trail: deque[Point2D] = field(default_factory=deque)
    # Filled by classify_tracks() once a region is known.
    in_region: bool = False
    time_to_enter: float | None = None
    entry_point: Point2D | None = None


class BoxTracker:
    def __init__(self, match_distance=0.4, max_age=0.5, window=0.4, trail_length=60):
        self.match_distance = match_distance
        self.max_age = max_age
        self.window = window
        self.trail_length = int(trail_length)
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
            # Copy the observation so later updates do not change its box/speed.
            # The trail is snapshotted as a list for the same reason.
            visible.append(replace(track, trail=deque(track.trail, maxlen=self.trail_length)))
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


# Region entry prediction

def predict_entry(center: Point2D, velocity: Point2D, polygon: list[Point2D],
                  horizon=3.0, step=0.1, min_speed=0.1) -> tuple[float, Point2D] | None:
    """Constant-velocity extrapolation: (seconds until entering polygon, entry point).

    None when the object is too slow to predict, already inside, or does not enter
    within the horizon.
    """
    if math.hypot(*velocity) < min_speed or inside(center, polygon):
        return None
    steps = int(round(horizon / step))
    for k in range(1, steps + 1):
        t = k * step
        point = (center[0] + velocity[0] * t, center[1] + velocity[1] * t)
        if inside(point, polygon):
            return t, point
    return None


def classify_tracks(tracks: list[Track], polygon: list[Point2D],
                    horizon=3.0, step=0.1, min_speed=0.1) -> list[Track]:
    """Set in_region and the entry prediction on each track (in place) and return them."""
    for track in tracks:
        track.in_region = inside(track.center, polygon)
        prediction = None if track.in_region else predict_entry(
            track.center, track.velocity, polygon, horizon, step, min_speed)
        track.time_to_enter, track.entry_point = prediction if prediction else (None, None)
    return tracks
