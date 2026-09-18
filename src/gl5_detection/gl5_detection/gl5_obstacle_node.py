from collections.abc import Callable
import json
import math
import os
from pathlib import Path
import tempfile
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Point32, PointStamped, PolygonStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, Marker, MarkerArray,
)
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from gl5_detection import detection_core as core


# 스캔을 받아 감지 코어를 돌리고 결과를 ROS/RViz로 내보내는 노드
class ObstacleNode(Node):
    # 파라미터, 상태, 토픽/서비스, 메뉴 준비 및 저장된 영역 불러오기
    def __init__(self):
        super().__init__('gl5_obstacle_detector')
        self.configure_parameters()
        self.initialize_state()
        self.create_ros_interfaces()
        if self.region_file.exists():
            try:
                self.load()
            except (ValueError, OSError, TypeError, KeyError) as exc:
                self.get_logger().error(f'Region file rejected: {exc}')
        self.setup_menu()
        self.create_timer(0.1, self.update_state_and_publish)
        self.get_logger().info('Publish Point: select vertices, then click near the first point to finish. '
                               'Region services: /gl5/region/{edit,undo,finish,cancel,clear,save,load,'
                               'learn_background}')
        self.publish_outputs()


    # 스캔 1개 처리: 배경 학습/차분 → 전체 군집화 → 추적 → 영역 판정
    def scan_callback(self, msg: LaserScan) -> None:
        now = time.monotonic()
        if not self.is_valid_scan(msg):
            self.reset_detection()
            return
        if self.last_valid_scan_time is None or now - self.last_valid_scan_time > self.timeout:
            self.occupancy_filter.reset()
            self.tracker.reset()
        self.last_valid_scan_time = now
        ranges = msg.ranges
        if self.background is not None:
            if self.background.learning:
                if self.background.observe(msg.ranges, msg.angle_min, msg.angle_increment):
                    self.get_logger().info('Background map learned; returns off the map are obstacles')
                return
            ranges = self.background.foreground(msg.ranges, msg.angle_min, msg.angle_increment)
            if not self.background.aligned:
                self.get_logger().warning(
                    f'Scan does not align with the background map (fitness '
                    f'{self.background.fitness:.2f}); press Learn Background if the sensor moved',
                    throttle_duration_sec=5.0)
        if not self.region or self.editing:
            return
        self.obstacle_clusters = core.cluster_scan(
            ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max, None, self.min_points, self.max_gap,
        )
        self.box_tracks = core.classify_tracks(
            self.tracker.update(self.obstacle_clusters, now), self.to_laser(self.region),
            self.predict_horizon, self.predict_step, self.min_predict_speed,
        )
        self.occupancy_filter.update(any(track.in_region for track in self.box_tracks), now)

    # 프레임, 각도, 거리 메타데이터와 유효 반사 존재 여부 검사
    def is_valid_scan(self, msg: LaserScan) -> bool:
        if msg.header.frame_id != self.frame or len(msg.ranges) < 2:
            return False
        metadata = (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        if not all(math.isfinite(value) for value in metadata):
            return False
        if msg.angle_increment <= 0 or msg.range_min < 0 or msg.range_max <= msg.range_min:
            return False
        return any(
            math.isfinite(distance) and distance > 0 and msg.range_min <= distance <= msg.range_max
            for distance in msg.ranges
        )

    # 0.1초마다 상태(EDITING/NO_REGION/NO_DATA/LEARNING/OCCUPIED/WARNING/CLEAR) 결정과 발행
    def update_state_and_publish(self) -> None:
        if self.editing:
            self.state = 'EDITING'
        elif not self.region:
            self.state = 'NO_REGION'
        elif self.last_valid_scan_time is None or time.monotonic() - self.last_valid_scan_time > self.timeout:
            self.state = 'NO_DATA'
            self.clear_detection_results()
        elif self.background is not None and self.background.learning:
            self.state = 'LEARNING'
        elif self.occupancy_filter.occupied:
            self.state = 'OCCUPIED'
        elif self.alerting_tracks():
            self.state = 'WARNING'
        else:
            self.state = 'CLEAR'
        self.publish_outputs()

    # 영역 안에 있거나 warning_time 안에 진입할 것으로 예측된 추적 목록
    def alerting_tracks(self) -> list[core.Track]:
        return [track for track in self.box_tracks if track.in_region or (
            track.time_to_enter is not None and track.time_to_enter <= self.warning_time)]

    # 군집, 추적, 점유 판정 초기화
    def clear_detection_results(self) -> None:
        self.obstacle_clusters = []
        self.box_tracks = []
        self.tracker.reset()
        self.occupancy_filter.reset()

    # 감지 결과와 마지막 수신 시각 초기화
    def reset_detection(self) -> None:
        self.clear_detection_results()
        self.last_valid_scan_time = None


    # 마커, 상태, 영역 폴리곤 일괄 발행
    def publish_outputs(self) -> None:
        self.publish_obstacle_markers()
        self.publish_detection_state()
        self.publish_region()


    # 배경 지도 좌표계 ← 현재 센서 좌표계 변환 (배경 없으면 항등)
    def anchor_from_laser(self) -> np.ndarray:
        if self.background is not None and self.background.ready:
            return self.background.pose
        return np.eye(3)

    # 지도 좌표계 → 현재 센서 좌표계 변환
    def to_laser(self, points: list[core.Point2D]) -> list[core.Point2D]:
        return core.transform_points(np.linalg.inv(self.anchor_from_laser()), points)

    # 센서 좌표계 → 지도 좌표계 변환
    def to_anchor(self, points: list[core.Point2D]) -> list[core.Point2D]:
        return core.transform_points(self.anchor_from_laser(), points)

    # 영역, 배경, 박스, 궤적, 예측선 MarkerArray 발행
    def publish_obstacle_markers(self) -> None:
        vertices = self.to_laser(self.draft if self.editing else self.region)
        background = []
        if self.background is not None and self.background.ready:
            background = self.background.contour()
        markers = self.visualization.build_markers(
            self.state, vertices, self.editing, self.box_tracks, self.obstacle_clusters,
            background=background,
        )
        self.marker_pub.publish(markers)

    # 상태 문자열과 감지/경고 Bool 발행
    def publish_detection_state(self) -> None:
        self.state_pub.publish(String(data=self.state))
        self.flag_pub.publish(Bool(data=self.state == 'OCCUPIED'))
        self.warning_pub.publish(Bool(data=self.state in ('WARNING', 'OCCUPIED')))

    # 현재 영역을 센서 좌표계 PolygonStamped로 발행
    def publish_region(self) -> None:
        self.region_pub.publish(make_region_polygon(
            self.frame, self.get_clock().now().to_msg(), self.to_laser(self.region)
        ))


    # 파라미터 읽기와 점유 필터, 추적기, 배경 모델, 시각화 객체 생성
    def configure_parameters(self) -> None:
        descriptor = ParameterDescriptor(read_only=True)

        # 읽기 전용 파라미터 선언과 값 반환
        def param(name, default):
            return self.declare_parameter(name, default, descriptor).value

        self.frame = param('frame_id', 'laser')
        self.region_file = Path(param('region_file', 'gl5_region.json')).expanduser()
        self.min_points = param('min_points', 5)
        self.max_gap = param('cluster_gap', 0.15)
        self.timeout = param('data_timeout', 1.0)
        self.close_radius = param('close_radius', 0.15)
        self.label_height = param('label_height', 0.14)
        match_distance = param('track_match_distance', 0.4)
        max_age = param('track_max_age', 0.5)
        speed_window = param('speed_window', 0.4)
        enter = param('enter_delay', 0.2)
        leave = param('exit_delay', 0.5)
        self.predict_horizon = param('predict_horizon', 3.0)
        self.predict_step = param('predict_step', 0.1)
        self.warning_time = param('warning_time', 2.0)
        self.min_predict_speed = param('min_predict_speed', 0.1)
        trail_length = param('trail_length', 60)
        background_enabled = param('background_enabled', True)
        learn_frames = param('background_learn_frames', 80)
        margin = param('background_margin', 0.15)
        ratio = param('background_ratio', 0.02)
        self.validate_parameters(match_distance, max_age, speed_window, enter, leave)
        if any(not math.isfinite(v) or v <= 0 for v in
               (self.predict_horizon, self.predict_step, self.warning_time)) or \
                not math.isfinite(self.min_predict_speed) or self.min_predict_speed < 0 or \
                trail_length < 1:
            raise ValueError('Invalid prediction parameters')
        self.occupancy_filter = core.Occupancy(enter, leave)
        self.background = None
        if background_enabled:
            self.background = core.BackgroundModel(learn_frames, margin, ratio)
            self.background.start_learning()
        self.tracker = core.BoxTracker(match_distance, max_age, speed_window, trail_length)
        self.visualization = RegionVisualization(
            self.frame, self.label_height, lambda: self.get_clock().now().to_msg(),
            self.warning_time,
        )

    # 파라미터 범위 검사
    def validate_parameters(self, match_distance, max_age, speed_window, enter, leave) -> None:
        if not self.frame or self.min_points < 1:
            raise ValueError('Invalid detector parameters')
        positive_values = (
            self.max_gap, self.timeout, self.close_radius,
            self.label_height, match_distance, max_age,
        )
        if any(not math.isfinite(value) or value <= 0 for value in positive_values):
            raise ValueError('Invalid detector parameters')
        if any(not math.isfinite(value) or value < 0 for value in (enter, leave)):
            raise ValueError('Invalid detector parameters')
        if not math.isfinite(speed_window) or speed_window < 0.2:
            raise ValueError('Invalid detector parameters')

    # 영역, 초안, 추적, 상태 변수 초기화
    def initialize_state(self) -> None:
        self.box_tracks: list[core.Track] = []
        self.region: list[core.Point2D] = []
        self.draft: list[core.Point2D] = []
        self.obstacle_clusters: list[list[core.Point2D]] = []
        self.editing = False
        self.last_valid_scan_time: float | None = None
        self.state = 'NO_REGION'
        self.menu_notice = ''

    # 퍼블리셔, 서브스크라이버, /gl5/region/* 서비스 생성
    def create_ros_interfaces(self) -> None:
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/gl5/obstacle_markers', qos)
        self.region_pub = self.create_publisher(PolygonStamped, '/gl5/region', qos)
        self.state_pub = self.create_publisher(String, '/gl5/obstacle_state', qos)
        self.flag_pub = self.create_publisher(Bool, '/gl5/obstacle_detected', qos)
        self.warning_pub = self.create_publisher(Bool, '/gl5/collision_warning', qos)
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        region_actions = (
            ('edit', self.edit), ('undo', self.undo), ('finish', self.finish),
            ('cancel', self.cancel), ('clear', self.clear), ('save', self.save), ('load', self.load),
            ('learn_background', self.learn_background),
        )
        self.region_services = [
            self.create_service(Trigger, '/gl5/region/' + name, self.make_service_callback(action))
            for name, action in region_actions
        ]


    # RViz 클릭 점을 지도 좌표로 변환해 초안 꼭짓점 추가 또는 영역 닫기
    def clicked_point_callback(self, msg: PointStamped) -> None:
        if msg.header.frame_id != self.frame:
            self.get_logger().warning(f'Click rejected: RViz Fixed Frame must be {self.frame}')
            return
        point = (msg.point.x, msg.point.y)
        if not all(math.isfinite(x) for x in point):
            return
        point = self.to_anchor([point])[0]
        if not self.editing:
            if self.region:
                self.get_logger().info('Region locked. Use region edit to draw a new region.')
                return
            self.edit()
        try:
            if len(self.draft) >= 3 and math.dist(point, self.draft[0]) <= self.close_radius:
                self.get_logger().info(self.finish())
            elif len(self.draft) >= 100:
                raise ValueError('Maximum 100 vertices')
            elif any(math.dist(point, p) < 1e-6 for p in self.draft):
                raise ValueError('Duplicate vertex; use finish to close the polygon')
            else:
                self.draft.append(point)
                self.get_logger().info(f'Vertex {len(self.draft)}: x={point[0]:.2f}, y={point[1]:.2f}')
        except (ValueError, OSError) as exc:
            self.get_logger().warning(str(exc))
        self.update_state_and_publish()

    # 영역 편집 시작
    def edit(self):
        self.draft = []
        self.editing = True
        self.reset_detection()
        return 'Editing: click perimeter vertices in RViz; click first vertex to finish'

    # 초안 마지막 꼭짓점 삭제
    def undo(self):
        if not self.editing or not self.draft:
            raise ValueError('No draft vertex to undo')
        self.draft.pop()
        return f'{len(self.draft)} draft vertices'

    # 초안 검증·저장 후 영역 확정
    def finish(self):
        if not self.editing:
            raise ValueError('Not editing')
        candidate = core.validate_polygon(self.draft)
        self.write_region(candidate)
        self.region = candidate
        self.draft = []
        self.editing = False
        self.reset_detection()
        return f'Region applied and saved: {self.region_file}'

    # 편집 취소, 이전 영역 복원
    def cancel(self):
        self.draft = []
        self.editing = False
        self.reset_detection()
        return 'Editing cancelled; previous region restored'

    # 영역과 저장 파일 비우기
    def clear(self):
        self.write_region([])
        self.region = []
        self.draft = []
        self.editing = False
        self.reset_detection()
        return 'Region cleared and saved'

    # 현재 영역 파일 저장
    def save(self):
        if self.editing:
            raise ValueError('Finish or cancel editing before saving')
        self.write_region(self.region)
        return f'Saved {self.region_file}'

    # 파일에서 영역 불러오기
    def load(self):
        candidate = read_region(self.region_file, self.frame)
        self.region = candidate
        self.draft = []
        self.editing = False
        self.reset_detection()
        return f'Loaded {self.region_file}'

    # 영역 파일 원자적 쓰기
    def write_region(self, region: list[core.Point2D]) -> None:
        write_region(self.region_file, self.frame, region)


    # 영역을 현재 센서 좌표계로 이동 후 배경 지도 재학습
    def learn_background(self):
        if self.background is None:
            raise ValueError('Background subtraction is disabled (background_enabled=false)')
        self.region, self.draft = self.to_laser(self.region), self.to_laser(self.draft)
        self.background.reset()
        self.background.start_learning()
        self.clear_detection_results()
        return (f'Learning background map from the next {self.background.learn_frames} scans; '
                'keep the area clear and the sensor still')


    # RViz 우클릭 메뉴(인터랙티브 마커) 생성
    def setup_menu(self):
        self.menu_server = InteractiveMarkerServer(self, '/gl5/region_menu')
        self.menu_handler = MenuHandler()
        for title, action in (
                ('Clear Region', self.clear),
                ('Finish Region', self.finish),
                ('Draw Region', self.edit),
                ('Undo Last Point', self.undo),
                ('Cancel Edit', self.cancel),
                ('Learn Background', self.learn_background)):
            self.menu_handler.insert(title, callback=self.make_menu_callback(action))
        menu = make_region_menu(self.frame)
        self.menu_server.insert(menu)
        self.menu_handler.apply(self.menu_server, menu.name)
        self.menu_server.applyChanges()

    # 메뉴 항목 콜백 생성
    def make_menu_callback(self, action):
        # 동작 실행 후 로그와 상태 반영
        def callback(feedback):
            try:
                message = action()
                self.menu_notice = ''
                self.get_logger().info(message)
            except (ValueError, OSError, TypeError, KeyError) as exc:
                self.menu_notice = str(exc)
                self.get_logger().warning(self.menu_notice)
            self.update_state_and_publish()
        return callback

    # Trigger 서비스 콜백 생성
    def make_service_callback(self, action):
        # 동작 실행 후 성공 여부와 메시지 응답
        def callback(request, response):
            try:
                response.message = action()
                response.success = True
            except (ValueError, OSError, TypeError, KeyError) as exc:
                response.success = False
                response.message = str(exc)
            self.update_state_and_publish()
            return response
        return callback


# 영역 JSON 파일 읽기와 꼭짓점 검증
def read_region(path: Path, frame_id: str) -> list[core.Point2D]:
    data = json.loads(path.read_text())
    if data['version'] != 1 or data['frame_id'] != frame_id:
        raise ValueError('Region version or frame_id mismatch')
    vertices = data['vertices']
    return [] if vertices == [] else core.validate_polygon(vertices)


# 임시 파일 쓰기 후 교체로 파일 손상 방지
def write_region(path: Path, frame_id: str, vertices: list[core.Point2D]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    filename = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w', dir=path.parent, prefix='.gl5-region-', delete=False
        ) as output:
            filename = output.name
            json.dump(
                {'version': 1, 'frame_id': frame_id, 'vertices': vertices},
                output,
                indent=2,
            )
            output.write('\n')
        os.replace(filename, path)
    finally:
        if filename and os.path.exists(filename):
            os.unlink(filename)


STATE_COLORS = {
    'CLEAR': (0.1, 0.9, 0.4, 1.0),
    'OCCUPIED': (1.0, 0.2, 0.1, 1.0),
    'EDITING': (1.0, 0.8, 0.1, 1.0),
    'NO_REGION': (1.0, 0.8, 0.1, 1.0),
    'NO_DATA': (0.6, 0.6, 0.6, 1.0),
    'LEARNING': (0.4, 0.7, 1.0, 1.0),
    'WARNING': (1.0, 0.6, 0.1, 1.0),
}
TRACK_COLORS = {
    'inside': (1.0, 0.1, 0.1, 1.0),
    'approaching': (1.0, 0.8, 0.1, 1.0),
    'outside': (0.2, 0.9, 0.4, 1.0),
}
TRAIL_COLOR = (0.3, 0.8, 1.0, 0.9)
PREDICTION_COLOR = (1.0, 0.8, 0.1, 0.9)
BACKGROUND_COLOR = (0.55, 0.55, 0.6, 0.8)


# 메뉴용 인터랙티브 마커 생성
def make_region_menu(frame_id: str) -> InteractiveMarker:
    menu = InteractiveMarker()
    menu.header.frame_id = frame_id
    menu.name = 'region_menu'
    menu.description = 'ROI MENU (right-click)'
    menu.pose.position.y = -0.7
    menu.pose.position.z = 0.15
    menu.pose.orientation.w = 1.0
    menu.scale = 0.5

    control = InteractiveMarkerControl()
    control.name = 'menu'
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.always_visible = True
    icon = Marker()
    icon.type = Marker.CUBE
    icon.pose.orientation.w = 1.0
    icon.scale.x = icon.scale.y = 0.35
    icon.scale.z = 0.08
    icon.color.r, icon.color.g, icon.color.b, icon.color.a = 0.1, 0.55, 1.0, 1.0
    control.markers.append(icon)
    menu.controls.append(control)
    return menu


# 꼭짓점 목록 → PolygonStamped
def make_region_polygon(
    frame_id: str, stamp: Time, region: list[core.Point2D]
) -> PolygonStamped:
    polygon = PolygonStamped()
    polygon.header.frame_id = frame_id
    polygon.header.stamp = stamp
    polygon.polygon.points = [
        Point32(x=float(x), y=float(y), z=0.0) for x, y in region
    ]
    return polygon


# RViz 마커 생성 담당
class RegionVisualization:
    # 프레임, 라벨 높이, 시각 함수, 경고 시간 저장
    def __init__(self, frame_id: str, label_height: float, timestamp: Callable[[], Time],
                 warning_time: float = 2.0):
        self.frame_id = frame_id
        self.label_height = label_height
        self.timestamp = timestamp
        self.warning_time = warning_time

    # 한 주기의 전체 마커(DELETEALL 포함) 조립
    def build_markers(
        self,
        state: str,
        vertices: list[core.Point2D],
        editing: bool,
        tracks: list[core.Track],
        clusters: list[list[core.Point2D]],
        background: list[core.Point2D] = (),
    ) -> MarkerArray:
        markers = [Marker(action=Marker.DELETEALL), self._selection_surface()]
        if background:
            markers.append(self._background_outline(background))
        if vertices:
            markers.extend(self._region_markers(vertices, editing, STATE_COLORS[state]))
        for track in tracks:
            markers.extend(self._obstacle_markers(track))
        markers.append(self._hit_points(clusters))
        return MarkerArray(markers=markers)

    # 프레임, 시각, 네임스페이스, 색이 채워진 빈 마커 생성
    def _marker(self, namespace, identifier, kind, color) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.timestamp()
        marker.ns = namespace
        marker.id = identifier
        marker.type = kind
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    @staticmethod
    # (x, y) 목록 → z=0.03인 Point 목록
    def _points(coordinates: list[core.Point2D]) -> list[Point]:
        return [Point(x=float(x), y=float(y), z=0.03) for x, y in coordinates]

    # 클릭용 바닥 투명 판
    def _selection_surface(self) -> Marker:
        floor = self._marker('selection_surface', 0, Marker.CUBE, (0.2, 0.3, 0.4, 0.12))
        floor.pose.position.z = -0.08
        floor.scale.x = floor.scale.y = 120.0
        floor.scale.z = 0.01
        return floor

    # 학습된 배경 지도 점(회색)
    def _background_outline(self, points: list[core.Point2D]) -> Marker:
        outline = self._marker('background', 0, Marker.POINTS, BACKGROUND_COLOR)
        outline.scale.x = outline.scale.y = 0.03
        outline.points = self._points(points)
        return outline

    # 영역 테두리 선과 꼭짓점 구
    def _region_markers(self, vertices, editing, color) -> list[Marker]:
        line = self._marker('region', 0, Marker.LINE_STRIP, color)
        line.scale.x = 0.035
        line.points = self._points(vertices + ([] if editing else vertices[:1]))
        dots = self._marker('vertices', 0, Marker.SPHERE_LIST, color)
        dots.scale.x = dots.scale.y = dots.scale.z = 0.08
        dots.points = self._points(vertices)
        return [line, dots]

    # 추적 상태 판정 (영역 안 / 진입 예정 / 그 외)
    def _track_status(self, track: core.Track) -> str:
        if track.in_region:
            return 'inside'
        if track.time_to_enter is not None and track.time_to_enter <= self.warning_time:
            return 'approaching'
        return 'outside'

    # 추적 박스, 라벨, 궤적, 예측선 마커
    def _obstacle_markers(self, track: core.Track) -> list[Marker]:
        x0, y0, x1, y1 = track.box
        color = TRACK_COLORS[self._track_status(track)]
        box = self._marker('obstacles', track.id, Marker.LINE_STRIP, color)
        box.scale.x = 0.025
        box.points = self._points([(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)])

        label = self._marker('obstacle_labels', track.id, Marker.TEXT_VIEW_FACING, color)
        label.scale.z = self.label_height
        label.pose.position.x = float((x0 + x1) / 2)
        label.pose.position.y = float(y1 + self.label_height + 0.04)
        label.pose.position.z = 0.08
        speed = '--' if track.speed is None else f'{track.speed:.2f}'
        label.text = f'#{track.id}\n{x1-x0:.2f}x{y1-y0:.2f}\n{speed}m/s'
        if track.time_to_enter is not None:
            label.text += f'\nin{track.time_to_enter:.1f}s'
        markers = [box, label]
        if len(track.trail) >= 2:
            trail = self._marker('trails', track.id, Marker.LINE_STRIP, TRAIL_COLOR)
            trail.scale.x = 0.02
            trail.points = self._points(list(track.trail))
            markers.append(trail)
        if track.entry_point is not None:
            markers.extend(self._prediction_markers(track))
        return markers

    # 현재 위치에서 예측 진입 지점까지의 점선과 진입 지점 구
    def _prediction_markers(self, track: core.Track) -> list[Marker]:
        path = self._marker('predictions', track.id, Marker.LINE_LIST, PREDICTION_COLOR)
        path.scale.x = 0.02
        (sx, sy), (ex, ey) = track.center, track.entry_point
        length = math.dist(track.center, track.entry_point)
        dashes = max(1, int(length / 0.2))
        segments = []
        for k in range(dashes):
            a, b = k / dashes, (k + 0.5) / dashes
            segments.append((sx + (ex - sx) * a, sy + (ey - sy) * a))
            segments.append((sx + (ex - sx) * b, sy + (ey - sy) * b))
        path.points = self._points(segments)
        entry = self._marker('entry_points', track.id, Marker.SPHERE, PREDICTION_COLOR)
        entry.scale.x = entry.scale.y = entry.scale.z = 0.12
        entry.pose.position.x, entry.pose.position.y = float(ex), float(ey)
        entry.pose.position.z = 0.03
        return [path, entry]

    # 군집에 포함된 전경 점(주황)
    def _hit_points(self, clusters: list[list[core.Point2D]]) -> Marker:
        hits = self._marker('hits', 0, Marker.POINTS, (1.0, 0.3, 0.1, 1.0))
        hits.scale.x = hits.scale.y = 0.045
        hits.points = self._points([point for cluster in clusters for point in cluster])
        return hits

# 노드 생성과 실행
def main():
    rclpy.init()
    node = None
    try:
        node = ObstacleNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
