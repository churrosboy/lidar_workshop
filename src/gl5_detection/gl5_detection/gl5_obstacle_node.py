#!/usr/bin/env python3
"""Interactive ROI selection and observed obstacle bounding boxes in the laser frame."""
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
from geometry_msgs.msg import Point, PointStamped, PolygonStamped, Point32
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from gl5_detection.roi_geometry import validate_polygon, clusters, bounds, Occupancy
from gl5_detection.roi_tracking import BoxTracker


class ObstacleNode(Node):
    def __init__(self):
        super().__init__('gl5_obstacle_detector')
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
        enter, leave = param('enter_delay', 0.2), param('exit_delay', 0.5)
        if not self.frame or self.min_points < 1 or any(not math.isfinite(v) or v <= 0 for v in
                (self.max_gap, self.timeout, self.close_radius, self.label_height, match_distance, max_age)) or any(
                not math.isfinite(v) or v < 0 for v in (enter, leave)) or not math.isfinite(speed_window) or speed_window < 0.2:
            raise ValueError('Invalid detector parameters')
        self.gate = Occupancy(enter, leave)
        self.tracker = BoxTracker(match_distance, max_age, speed_window)
        self.box_tracks = []
        self.region, self.draft, self.groups = [], [], []
        self.editing = False
        self.last_scan = None
        self.state = 'NO_REGION'
        self.menu_notice = ''
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/gl5/obstacle_markers', qos)
        self.region_pub = self.create_publisher(PolygonStamped, '/gl5/region', qos)
        self.state_pub = self.create_publisher(String, '/gl5/obstacle_state', qos)
        self.flag_pub = self.create_publisher(Bool, '/gl5/obstacle_detected', qos)
        self.create_subscription(PointStamped, '/clicked_point', self.clicked, 10)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.region_services = [self.create_service(Trigger, '/gl5/region/' + name,
                        self.service(action)) for name, action in (
            ('edit', self.edit), ('undo', self.undo), ('finish', self.finish),
            ('cancel', self.cancel), ('clear', self.clear), ('save', self.save), ('load', self.load))]
        if self.region_file.exists():
            try:
                self.load()
            except (ValueError, OSError, TypeError, KeyError) as exc:
                self.get_logger().error(f'Region file rejected: {exc}')
        self.setup_menu()
        self.create_timer(0.1, self.tick)
        self.get_logger().info('Publish Point: select vertices, then click near the first point to finish. '
                               'Region services: /gl5/region/{edit,undo,finish,cancel,clear,save,load}')
        self.publish()

    def setup_menu(self):
        self.menu_server = InteractiveMarkerServer(self, '/gl5/region_menu')
        self.menu_handler = MenuHandler()
        for title, action in (
                ('Clear Region', self.clear),
                ('Finish Region', self.finish),
                ('Draw Region', self.edit),
                ('Undo Last Point', self.undo),
                ('Cancel Edit', self.cancel)):
            self.menu_handler.insert(title, callback=self.menu_action(action))
        menu = InteractiveMarker()
        menu.header.frame_id = self.frame
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
        self.menu_server.insert(menu)
        self.menu_handler.apply(self.menu_server, menu.name)
        self.menu_server.applyChanges()

    def menu_action(self, action):
        def callback(feedback):
            try:
                message = action()
                self.menu_notice = ''
                self.get_logger().info(message)
            except (ValueError, OSError, TypeError, KeyError) as exc:
                self.menu_notice = str(exc)
                self.get_logger().warning(self.menu_notice)
            self.tick()
        return callback

    def service(self, action):
        def callback(request, response):
            try:
                response.message = action()
                response.success = True
            except (ValueError, OSError, TypeError, KeyError) as exc:
                response.success = False
                response.message = str(exc)
            self.tick()
            return response
        return callback

    def reset_detection(self):
        self.groups = []
        self.box_tracks = []
        self.tracker.reset()
        self.gate.reset()
        self.last_scan = None

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
        candidate = validate_polygon(self.draft)
        # Commit only after a successful atomic save; an invalid draft never replaces the ROI.
        self.write_region(candidate)
        self.region, self.draft, self.editing = candidate, [], False
        self.reset_detection()
        return f'Region applied and saved: {self.region_file}'

    def cancel(self):
        self.draft, self.editing = [], False
        self.reset_detection()
        return 'Editing cancelled; previous region restored'

    def clear(self):
        # A saved empty region makes clear persist across restarts.
        self.write_region([])
        self.region, self.draft, self.editing = [], [], False
        self.reset_detection()
        return 'Region cleared and saved'

    def write_region(self, region):
        self.region_file.parent.mkdir(parents=True, exist_ok=True)
        filename = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=self.region_file.parent,
                                             prefix='.gl5-region-', delete=False) as out:
                filename = out.name
                json.dump({'version': 1, 'frame_id': self.frame, 'vertices': region}, out, indent=2)
                out.write('\n')
            os.replace(filename, self.region_file)
        finally:
            if filename and os.path.exists(filename):
                os.unlink(filename)

    def save(self):
        if self.editing:
            raise ValueError('Finish or cancel editing before saving')
        self.write_region(self.region)
        return f'Saved {self.region_file}'

    def load(self):
        data = json.loads(self.region_file.read_text())
        if data['version'] != 1 or data['frame_id'] != self.frame:
            raise ValueError('Region version or frame_id mismatch')
        region = data['vertices']
        candidate = [] if region == [] else validate_polygon(region)
        self.region, self.draft, self.editing = candidate, [], False
        self.reset_detection()
        return f'Loaded {self.region_file}'

    def clicked(self, msg):
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
        self.tick()

    def scan(self, msg):
        now = time.monotonic()
        if (msg.header.frame_id != self.frame or len(msg.ranges) < 2 or
            not all(math.isfinite(v) for v in (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)) or
            msg.angle_increment <= 0 or msg.range_min < 0 or msg.range_max <= msg.range_min or
            not any(math.isfinite(r) and r > 0 and msg.range_min <= r <= msg.range_max for r in msg.ranges)):
            self.last_scan = None
            self.groups = []
            self.box_tracks = []
            self.tracker.reset()
            self.gate.reset()
            return
        if self.last_scan is None or now - self.last_scan > self.timeout:
            self.gate.reset()
            self.tracker.reset()
        self.last_scan = now
        if not self.region or self.editing:
            return
        self.groups = clusters(msg.ranges, msg.angle_min, msg.angle_increment,
                               msg.range_min, msg.range_max, self.region, self.min_points, self.max_gap)
        self.box_tracks = self.tracker.update(self.groups, now)
        self.gate.update(bool(self.groups), now)

    def tick(self):
        if self.editing:
            self.state = 'EDITING'
        elif not self.region:
            self.state = 'NO_REGION'
        elif self.last_scan is None or time.monotonic() - self.last_scan > self.timeout:
            self.state = 'NO_DATA'
            self.groups = []
            self.box_tracks = []
            self.tracker.reset()
            self.gate.reset()
        else:
            self.state = 'OCCUPIED' if self.gate.occupied else 'CLEAR'
        self.publish()

    def marker(self, ns, ident, kind, color):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type = ns, ident, kind
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def publish(self):
        colors = {'CLEAR': (0.1, 0.9, 0.4, 1.0), 'OCCUPIED': (1.0, 0.2, 0.1, 1.0),
                  'EDITING': (1.0, 0.8, 0.1, 1.0), 'NO_REGION': (1.0, 0.8, 0.1, 1.0),
                  'NO_DATA': (0.6, 0.6, 0.6, 1.0)}
        color = colors[self.state]
        delete = Marker(action=Marker.DELETEALL)
        markers = [delete]
        # PublishPoint needs selectable geometry even where there are no laser returns.
        floor = self.marker('selection_surface', 0, Marker.CUBE, (0.2, 0.3, 0.4, 0.12))
        floor.pose.position.z = -0.08
        floor.scale.x = floor.scale.y = 120.0
        floor.scale.z = 0.01
        markers.append(floor)
        vertices = self.draft if self.editing else self.region
        def points(coords):
            return [Point(x=float(x), y=float(y), z=0.03) for x, y in coords]
        if vertices:
            line = self.marker('region', 0, Marker.LINE_STRIP, color)
            line.scale.x = 0.035
            line.points = points(vertices + ([] if self.editing else vertices[:1]))
            markers.append(line)
            dots = self.marker('vertices', 0, Marker.SPHERE_LIST, color)
            dots.scale.x = dots.scale.y = dots.scale.z = 0.08
            dots.points = points(vertices)
            markers.append(dots)
        for track in self.box_tracks:
            x0, y0, x1, y1 = track['box']
            box = self.marker('obstacles', track['id'], Marker.LINE_STRIP, (1.0, 0.1, 0.1, 1.0))
            box.scale.x = 0.025
            box.points = points([(x0,y0), (x1,y0), (x1,y1), (x0,y1), (x0,y0)])
            markers.append(box)
            label = self.marker('obstacle_labels', track['id'], Marker.TEXT_VIEW_FACING,
                                (1.0, 0.1, 0.1, 1.0))
            label.scale.z = self.label_height
            label.pose.position.x = float((x0+x1)/2)
            label.pose.position.y = float(y1 + self.label_height + 0.04)
            label.pose.position.z = 0.08
            speed = '--' if track['speed'] is None else f"{track['speed']:.2f}"
            # RViz's text renderer can give ASCII spaces an excessive width.
            label.text = f"#{track['id']}\n{x1-x0:.2f}x{y1-y0:.2f}\n{speed}m/s"
            markers.append(label)
        hits = self.marker('hits', 0, Marker.POINTS, (1.0, 0.3, 0.1, 1.0))
        hits.scale.x = hits.scale.y = 0.045
        hits.points = points([p for group in self.groups for p in group])
        markers.append(hits)
        label = self.marker('status', 0, Marker.TEXT_VIEW_FACING, color)
        label.scale.z = 0.18
        label.pose.position.x = min((p[0] for p in vertices), default=0.0)
        label.pose.position.y = max((p[1] for p in vertices), default=0.0) + 0.3
        label.pose.position.z = 0.1
        label.text = f'{self.state} | {len(self.groups)} clusters'
        if self.state in ('NO_REGION', 'EDITING'):
            label.text += ' | Publish Point: vertices, then first point'
        if self.menu_notice:
            label.text += '\n' + self.menu_notice
        # State and action guidance are shown in the docked RViz RegionPanel.
        self.marker_pub.publish(MarkerArray(markers=markers))
        self.state_pub.publish(String(data=self.state))
        self.flag_pub.publish(Bool(data=self.state == 'OCCUPIED'))
        polygon = PolygonStamped()
        polygon.header.frame_id = self.frame
        polygon.header.stamp = self.get_clock().now().to_msg()
        polygon.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x,y in self.region]
        self.region_pub.publish(polygon)


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
