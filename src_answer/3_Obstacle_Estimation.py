# 등속 직진 가정의 폴리곤 최초 진입 시각과 지점 계산
def predict_entry(center: Point2D, velocity: Point2D, polygon: list[Point2D],
                  horizon=3.0, step=0.1, min_speed=0.1) -> tuple[float, Point2D] | None:
    if math.hypot(*velocity) < min_speed or inside(center, polygon):
        return None
    steps = int(round(horizon / step))
    for k in range(1, steps + 1):
        t = k * step
        point = (center[0] + velocity[0] * t, center[1] + velocity[1] * t)
        if inside(point, polygon):
            return t, point
    return None