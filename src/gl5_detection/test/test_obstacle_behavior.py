"""Replay workshop interactions against outputs captured before refactoring."""
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import LaserScan

from gl5_detection.gl5_obstacle_node import (
    ObstacleNode, RegionVisualization, make_region_menu,
)
from gl5_detection.detection_core import BoxTracker, Occupancy


class Publisher:
    def publish(self, message):
        self.message = message_to_ordereddict(message)


class DetectorHarness(ObstacleNode):
    """Exercise real callbacks without DDS, a sensor, or a running RViz."""

    def __init__(self, region_file):
        self.frame = 'laser'
        self.region_file = region_file
        self.min_points = 5
        self.max_gap = 0.15
        self.timeout = 1.0
        self.close_radius = 0.15
        self.label_height = 0.14
        self.occupancy_filter = Occupancy(0.2, 0.5)
        self.tracker = BoxTracker(0.4, 0.5, 0.4)
        self.box_tracks = []
        self.region, self.draft, self.obstacle_clusters = [], [], []
        self.editing = False
        self.last_valid_scan_time = None
        self.scan_geometry = None
        self.background = None  # golden fixture predates background subtraction
        self.state = 'NO_REGION'
        self.menu_notice = ''
        self.visualization = RegionVisualization(
            self.frame, self.label_height, lambda: self.get_clock().now().to_msg()
        )
        self.marker_pub = Publisher()
        self.region_pub = Publisher()
        self.state_pub = Publisher()
        self.flag_pub = Publisher()

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=123)))

    def get_logger(self):
        return SimpleNamespace(info=lambda _: None, warning=lambda _: None)


def scan(ranges, frame='laser'):
    message = LaserScan(
        angle_min=-0.05,
        angle_increment=0.01,
        range_min=0.0,
        range_max=60.0,
        ranges=ranges,
    )
    message.header.frame_id = frame
    return message


def replay(region_file):
    node = DetectorHarness(region_file)
    snapshots = []

    def record(name, now):
        with patch('gl5_detection.gl5_obstacle_node.time.monotonic', return_value=now):
            node.update_state_and_publish()
        snapshots.append({
            'name': name,
            'markers': node.marker_pub.message,
            'polygon': node.region_pub.message,
            'state': node.state_pub.message,
            'detected': node.flag_pub.message,
            'draft': node.draft.copy(),
            'saved': json.loads(region_file.read_text()) if region_file.exists() else None,
        })

    def receive(ranges, now, frame='laser'):
        with patch('gl5_detection.gl5_obstacle_node.time.monotonic', return_value=now):
            node.scan_callback(scan(ranges, frame))

    record('initial', 0.0)
    node.edit()
    record('empty draft', 0.0)
    for x, y in [(0.2, -0.5), (2.0, -0.5), (2.0, 0.5), (0.2, 0.5)]:
        point = PointStamped()
        point.header.frame_id = 'laser'
        point.point.x, point.point.y = x, y
        with patch('gl5_detection.gl5_obstacle_node.time.monotonic', return_value=0.0):
            node.clicked_point_callback(point)
    record('draft vertices', 0.0)
    node.finish()
    record('saved region without scan', 0.0)
    receive([1.0] * 5 + [math.inf] + [1.5] * 5, 1.0)
    record('entry pending', 1.0)
    for index in range(1, 9):
        now = 1.0 + index * 0.05
        receive([1.0 + index * 0.01] * 5 + [math.inf] + [1.5] * 5, now)
    record('occupied with speed', 1.4)
    receive([3.0] * 11, 1.5)
    record('exit pending', 1.5)
    receive([3.0] * 11, 2.01)
    record('clear', 2.01)
    receive([math.inf] * 11, 2.1)
    record('all returns invalid', 2.1)
    receive([1.0] * 11, 2.2)
    receive([1.0] * 11, 2.41)
    record('recovered', 2.41)
    record('timeout', 3.42)
    receive([1.0] * 11, 3.5, frame='wrong_frame')
    record('wrong frame', 3.5)
    node.edit()
    node.draft = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    node.undo()
    record('undo', 3.6)
    node.cancel()
    record('cancel restores region', 3.6)
    node.load()
    record('load', 3.6)
    node.clear()
    record('clear persists', 3.6)
    node.load()
    record('load empty region', 3.6)
    # JSON normalizes coordinate tuples just like the checked-in fixture.
    return json.loads(json.dumps(snapshots))


class ObstacleBehaviorTest(unittest.TestCase):
    def test_interactive_menu_matches_original(self):
        fixture = Path(__file__).parent / 'fixtures' / 'region_menu.json'
        self.assertEqual(
            message_to_ordereddict(make_region_menu('laser')),
            json.loads(fixture.read_text()),
        )

    def test_rviz_messages_and_state_match_original(self):
        fixture = Path(__file__).parent / 'fixtures' / 'obstacle_behavior.json'
        with tempfile.TemporaryDirectory() as directory:
            actual = replay(Path(directory) / 'region.json')
        expected = json.loads(fixture.read_text())
        self.assertEqual(len(actual), len(expected))
        for result, original in zip(actual, expected):
            with self.subTest(state=original['name']):
                self.assertEqual(result, original)

    def test_failed_save_preserves_applied_region_and_draft(self):
        with tempfile.TemporaryDirectory() as directory:
            node = DetectorHarness(Path(directory) / 'region.json')
            original = [(0.0, 0.0), (2.0, 0.0), (0.0, 2.0)]
            node.region = original.copy()
            node.save()
            saved = node.region_file.read_bytes()
            node.edit()
            draft = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
            node.draft = draft.copy()
            with patch.object(node, 'write_region', side_effect=OSError('disk unavailable')):
                with self.assertRaises(OSError):
                    node.finish()
                with self.assertRaises(OSError):
                    node.clear()
            self.assertEqual(node.region, original)
            self.assertEqual(node.draft, draft)
            self.assertTrue(node.editing)
            self.assertEqual(node.region_file.read_bytes(), saved)


if __name__ == '__main__':
    unittest.main()
