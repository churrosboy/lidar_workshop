import math

from gl5_detection.detection_core import Point2D, Track, inside, mark_in_region


# 등속 직진 가정의 폴리곤 최초 진입 시각과 지점 계산
def predict_entry(center: Point2D, velocity: Point2D, polygon: list[Point2D],
                  horizon=3.0, step=0.1, min_speed=0.1) -> tuple[float, Point2D] | None:
    # 너무 느린 물체는 예측하지 않습니다 (정지 물체 주변에서 깜빡이는 것을 막습니다).
    # 이미 영역 안에 있으면 "진입 예측"의 대상이 아닙니다.
    if math.hypot(*velocity) < min_speed or inside(center, polygon):
        return None
    steps = int(round(horizon / step))
    # ========================== Insert Your Code ==========================
    # 물체가 지금 속도 그대로 방향을 바꾸지 않고 직진한다고 가정하고,
    # step 초 간격으로 steps 번까지 앞날을 미리 찍어 봅니다.
    #
    #   k = 1, 2, ..., steps 에 대해
    #     t = k * step                      (지금부터 t 초 뒤)
    #     point = center + velocity * t     (그때의 위치. x, y 를 각각 계산합니다)
    #     그 point 가 inside(point, polygon) 이면 처음으로 영역에 들어간 순간이므로
    #     (t, point) 를 반환하고 끝냅니다.
    #
    # horizon 안에 한 번도 들어오지 않으면 None 을 반환합니다.

    raise NotImplementedError('3단계 predict_entry')
    # ========================== Insert Your Code ==========================


# 각 추적의 영역 안 여부와 진입 예측 채우기
def classify_tracks(tracks: list[Track], polygon: list[Point2D],
                    horizon=3.0, step=0.1, min_speed=0.1) -> list[Track]:
    for track in mark_in_region(tracks, polygon):
        prediction = None if track.in_region else predict_entry(
            track.center, track.velocity, polygon, horizon, step, min_speed)
        track.time_to_enter, track.entry_point = prediction if prediction else (None, None)
    return tracks
