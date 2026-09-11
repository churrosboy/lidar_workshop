"""ROS-independent polygon, scan clustering and temporal occupancy logic."""
import math

EPS = 1e-9


def cross(a, b, p):
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])


def on_segment(a, b, p):
    return abs(cross(a, b, p)) <= EPS and all(
        min(a[i], b[i]) - EPS <= p[i] <= max(a[i], b[i]) + EPS for i in (0, 1))


def intersects(a, b, c, d):
    return (cross(a, b, c) * cross(a, b, d) < 0 and
            cross(c, d, a) * cross(c, d, b) < 0) or any((
        on_segment(a, b, c), on_segment(a, b, d),
        on_segment(c, d, a), on_segment(c, d, b)))


def validate_polygon(vertices):
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


def inside(point, polygon):
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


def clusters(ranges, angle_min, angle_increment, range_min, range_max, polygon,
             min_points=5, max_gap=0.15):
    """Consecutive in-ROI beams form a cluster; missing/outside beams break it."""
    groups, current = [], []
    def flush():
        if len(current) >= min_points:
            groups.append(current.copy())
        current.clear()
    for i, distance in enumerate(ranges):
        if not math.isfinite(distance) or distance <= 0 or not range_min <= distance <= range_max:
            flush()
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


def bounds(points):
    return (min(p[0] for p in points), min(p[1] for p in points),
            max(p[0] for p in points), max(p[1] for p in points))


class Occupancy:
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
