import math

from gl5_detection.detection_core import Point2D, Track, inside, mark_in_region


# 등속 직진 가정의 폴리곤 최초 진입 시각과 지점 계산
def predict_entry(center: Point2D, velocity: Point2D, polygon: list[Point2D],
                  horizon=3.0, step=0.1, min_speed=0.1) -> tuple[float, Point2D] | None:
    # ===== [3단계 실습] 진입 예측을 채우세요 =====
    # 입력: center=(x, y) 현재 위치, velocity=(vx, vy) m/s, polygon=영역 꼭짓점 목록
    #       horizon=예측 최대 시간(s), step=시간 간격(s), min_speed=이보다 느리면 예측 안 함
    # 출력: (진입까지 걸리는 시간 t, 진입 지점 (x, y)) 또는 None
    # 순서:
    #  1. 속력(math.hypot(vx, vy))이 min_speed 미만이거나 center가 이미 영역 안(inside)이면 None
    #  2. t = step, 2*step, ... horizon까지 등속 직진: point = center + velocity * t
    #  3. 처음으로 inside(point, polygon)이 True가 되는 t와 point를 반환
    #  4. horizon 안에 들어오지 않으면 None
    # 채점: pytest test/test_prediction.py
    raise NotImplementedError('3단계: predict_entry를 채우세요')


# 각 추적의 영역 안 여부와 진입 예측 채우기
def classify_tracks(tracks: list[Track], polygon: list[Point2D],
                    horizon=3.0, step=0.1, min_speed=0.1) -> list[Track]:
    for track in mark_in_region(tracks, polygon):
        prediction = None if track.in_region else predict_entry(
            track.center, track.velocity, polygon, horizon, step, min_speed)
        track.time_to_enter, track.entry_point = prediction if prediction else (None, None)
    return tracks
