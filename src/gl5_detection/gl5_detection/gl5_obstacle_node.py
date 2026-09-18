#!/usr/bin/env python3
"""ROS adapter for GL5 obstacle detection and interactive region editing.

Read scan_callback() for the processing flow and publish_outputs() for ROS output.
The calculations live in detection_core.py; storage and RViz helpers are below.
"""
from collections.abc import Callable
import json
import math
import os
from pathlib import Path
import tempfile
import time

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


class ObstacleNode(Node):
    """Receive scans, call the detection core and publish results for ROS/RViz."""

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

    # Scan processing and state

    def scan_callback(self, msg: LaserScan) -> None:
        now = time.monotonic()
        if not self.is_valid_scan(msg):
            self.reset_detection()
            return
        if self.last_valid_scan_time is None or now - self.last_valid_scan_time > self.timeout:
            self.occupancy_filter.reset()
            self.tracker.reset()
        self.last_valid_scan_time = now
        self.scan_geometry = (msg.angle_min, msg.angle_increment)
        ranges = msg.ranges
        if self.background is not None:
            # Learning runs even without a region so the background outline is visible.
            if self.background.learning:
                if self.background.observe(msg.ranges):
                    self.get_logger().info('Background learned; closer returns are now obstacles')
                return
            ranges = self.background.foreground(msg.ranges)
            if self.background.learning:  # beam count changed (other sensor): relearning
                return
        if not self.region or self.editing:
            return
        self.obstacle_clusters = core.cluster_scan(
            ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max, self.region, self.min_points, self.max_gap,
        )
        self.box_tracks = self.tracker.update(self.obstacle_clusters, now)
        self.occupancy_filter.update(bool(self.obstacle_clusters), now)

    def is_valid_scan(self, msg: LaserScan) -> bool:
        if msg.header.frame_id != self.frame or len(msg.ranges) < 2:
            return False
        metadata = (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        if not all(math.isfinite(value) for value in metadata):
            return False
        if msg.angle_increment <= 0 or msg.range_min < 0 or msg.range_max <= msg.range_min:
            return False
        # A received frame with no valid return is NO_DATA, not an empty region.
        return any(
            math.isfinite(distance) and distance > 0 and msg.range_min <= distance <= msg.range_max
            for distance in msg.ranges
        )

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
        else:
            self.state = 'OCCUPIED' if self.occupancy_filter.occupied else 'CLEAR'
        self.publish_outputs()

    def clear_detection_results(self) -> None:
        self.obstacle_clusters = []
        self.box_tracks = []
        self.tracker.reset()
        self.occupancy_filter.reset()

    def reset_detection(self) -> None:
        self.clear_detection_results()
        self.last_valid_scan_time = None

    # ROS output

    def publish_outputs(self) -> None:
        self.publish_obstacle_markers()
        self.publish_detection_state()
        self.publish_region()

    def publish_obstacle_markers(self) -> None:
        vertices = self.draft if self.editing else self.region
        background = []
        if self.background is not None and self.background.ready and self.scan_geometry:
            background = self.background.contour(*self.scan_geometry)
        markers = self.visualization.build_markers(
            self.state, vertices, self.editing, self.box_tracks, self.obstacle_clusters,
            background=background,
        )
        self.marker_pub.publish(markers)

    def publish_detection_state(self) -> None:
        self.state_pub.publish(String(data=self.state))
        self.flag_pub.publish(Bool(data=self.state == 'OCCUPIED'))

    def publish_region(self) -> None:
        self.region_pub.publish(make_region_polygon(
            self.frame, self.get_clock().now().to_msg(), self.region
        ))

    # Parameters and ROS setup

    def configure_parameters(self) -> None:
        descriptor = ParameterDescriptor(read_only=True)

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
        background_enabled = param('background_enabled', True)
        learn_frames = param('background_learn_frames', 80)
        margin = param('background_margin', 0.15)
        ratio = param('background_ratio', 0.02)
        self.validate_parameters(match_distance, max_age, speed_window, enter, leave)
        self.occupancy_filter = core.Occupancy(enter, leave)
        # None disables background subtraction; otherwise learning starts with the first scans.
        self.background = None
        if background_enabled:
            self.background = core.BackgroundModel(learn_frames, margin, ratio)
            self.background.start_learning()
        self.tracker = core.BoxTracker(match_distance, max_age, speed_window)
        self.visualization = RegionVisualization(
            self.frame, self.label_height, lambda: self.get_clock().now().to_msg()
        )

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

    def initialize_state(self) -> None:
        self.box_tracks: list[core.Track] = []
        self.region: list[core.Point2D] = []
        self.draft: list[core.Point2D] = []
        self.obstacle_clusters: list[list[core.Point2D]] = []
        self.editing = False
        self.last_valid_scan_time: float | None = None
        self.scan_geometry: tuple[float, float] | None = None
        self.state = 'NO_REGION'
        self.menu_notice = ''

    def create_ros_interfaces(self) -> None:
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/gl5/obstacle_markers', qos)
        self.region_pub = self.create_publisher(PolygonStamped, '/gl5/region', qos)
        self.state_pub = self.create_publisher(String, '/gl5/obstacle_state', qos)
        self.flag_pub = self.create_publisher(Bool, '/gl5/obstacle_detected', qos)
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

    # Region editing and persistence

    def clicked_point_callback(self, msg: PointStamped) -> None:
        if msg.header.frame_id != self.frame:
            self.get_logger().warning(f'Click rejected: RViz Fixed Frame must be {self.frame}')
            return
        point = (msg.point.x, msg.point.y)
        if not all(math.isfinite(x) for x in point):
            return
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

    def edit(self):
        self.draft = []
        self.editing = True
        self.reset_detection()
        return 'Editing: click perimeter vertices in RViz; click first vertex to finish'

    def undo(self):
        if not self.editing or not self.draft:
            raise ValueError('No draft vertex to undo')
        self.draft.pop()
        return f'{len(self.draft)} draft vertices'

    def finish(self):
        if not self.editing:
            raise ValueError('Not editing')
        candidate = core.validate_polygon(self.draft)
        # Commit only after a successful atomic save; an invalid draft never replaces the ROI.
        self.write_region(candidate)
        self.region = candidate
        self.draft = []
        self.editing = False
        self.reset_detection()
        return f'Region applied and saved: {self.region_file}'

    def cancel(self):
        self.draft = []
        self.editing = False
        self.reset_detection()
        return 'Editing cancelled; previous region restored'

    def clear(self):
        # A saved empty region makes clear persist across restarts.
        self.write_region([])
        self.region = []
        self.draft = []
        self.editing = False
        self.reset_detection()
        return 'Region cleared and saved'

    def save(self):
        if self.editing:
            raise ValueError('Finish or cancel editing before saving')
        self.write_region(self.region)
        return f'Saved {self.region_file}'

    def load(self):
        candidate = read_region(self.region_file, self.frame)
        self.region = candidate
        self.draft = []
        self.editing = False
        self.reset_detection()
        return f'Loaded {self.region_file}'

    def write_region(self, region: list[core.Point2D]) -> None:
        write_region(self.region_file, self.frame, region)

    # Background subtraction

    def learn_background(self):
        if self.background is None:
            raise ValueError('Background subtraction is disabled (background_enabled=false)')
        self.background.start_learning()
        self.clear_detection_results()
        return (f'Learning background from the next {self.background.learn_frames} scans; '
                'keep the area clear')

    # RViz menu and service callbacks

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

    def make_menu_callback(self, action):
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

    def make_service_callback(self, action):
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


# Region file persistence

def read_region(path: Path, frame_id: str) -> list[core.Point2D]:
    data = json.loads(path.read_text())
    if data['version'] != 1 or data['frame_id'] != frame_id:
        raise ValueError('Region version or frame_id mismatch')
    vertices = data['vertices']
    return [] if vertices == [] else core.validate_polygon(vertices)


def write_region(path: Path, frame_id: str, vertices: list[core.Point2D]) -> None:
    """Keep the previous file intact if writing the replacement fails."""
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


# RViz message construction

STATE_COLORS = {
    'CLEAR': (0.1, 0.9, 0.4, 1.0),
    'OCCUPIED': (1.0, 0.2, 0.1, 1.0),
    'EDITING': (1.0, 0.8, 0.1, 1.0),
    'NO_REGION': (1.0, 0.8, 0.1, 1.0),
    'NO_DATA': (0.6, 0.6, 0.6, 1.0),
    'LEARNING': (0.4, 0.7, 1.0, 1.0),
}
OBSTACLE_COLOR = (1.0, 0.1, 0.1, 1.0)
BACKGROUND_COLOR = (0.55, 0.55, 0.6, 0.8)


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


class RegionVisualization:
    def __init__(self, frame_id: str, label_height: float, timestamp: Callable[[], Time]):
        self.frame_id = frame_id
        self.label_height = label_height
        self.timestamp = timestamp

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
        # Status and action guidance live in RegionPanel, not in a scene label.
        return MarkerArray(markers=markers)

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
    def _points(coordinates: list[core.Point2D]) -> list[Point]:
        return [Point(x=float(x), y=float(y), z=0.03) for x, y in coordinates]

    def _selection_surface(self) -> Marker:
        # PublishPoint needs selectable geometry even where there are no returns.
        floor = self._marker('selection_surface', 0, Marker.CUBE, (0.2, 0.3, 0.4, 0.12))
        floor.pose.position.z = -0.08
        floor.scale.x = floor.scale.y = 120.0
        floor.scale.z = 0.01
        return floor

    def _background_outline(self, points: list[core.Point2D]) -> Marker:
        outline = self._marker('background', 0, Marker.POINTS, BACKGROUND_COLOR)
        outline.scale.x = outline.scale.y = 0.03
        outline.points = self._points(points)
        return outline

    def _region_markers(self, vertices, editing, color) -> list[Marker]:
        line = self._marker('region', 0, Marker.LINE_STRIP, color)
        line.scale.x = 0.035
        line.points = self._points(vertices + ([] if editing else vertices[:1]))
        dots = self._marker('vertices', 0, Marker.SPHERE_LIST, color)
        dots.scale.x = dots.scale.y = dots.scale.z = 0.08
        dots.points = self._points(vertices)
        return [line, dots]

    def _obstacle_markers(self, track: core.Track) -> list[Marker]:
        x0, y0, x1, y1 = track.box
        box = self._marker('obstacles', track.id, Marker.LINE_STRIP, OBSTACLE_COLOR)
        box.scale.x = 0.025
        box.points = self._points([(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)])

        label = self._marker('obstacle_labels', track.id, Marker.TEXT_VIEW_FACING, OBSTACLE_COLOR)
        label.scale.z = self.label_height
        label.pose.position.x = float((x0 + x1) / 2)
        label.pose.position.y = float(y1 + self.label_height + 0.04)
        label.pose.position.z = 0.08
        speed = '--' if track.speed is None else f'{track.speed:.2f}'
        # RViz's text renderer can give ASCII spaces an excessive width.
        label.text = f'#{track.id}\n{x1-x0:.2f}x{y1-y0:.2f}\n{speed}m/s'
        return [box, label]

    def _hit_points(self, clusters: list[list[core.Point2D]]) -> Marker:
        hits = self._marker('hits', 0, Marker.POINTS, (1.0, 0.3, 0.1, 1.0))
        hits.scale.x = hits.scale.y = 0.045
        hits.points = self._points([point for cluster in clusters for point in cluster])
        return hits

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
