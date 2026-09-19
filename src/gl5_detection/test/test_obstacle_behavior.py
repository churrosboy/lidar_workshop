import json
import math
import os
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
from gl5_detection import prediction


class Publisher:
    def publish(self, message):
        self.message = message_to_ordereddict(message)


class DetectorHarness(ObstacleNode):
    def __init__(self, region_file):
        self.frame = 'laser'
        self.region_file = region_file
        self.min_points = 5
        self.max_gap = 0.15
        self.timeout = 1.0
        self.close_radius = 0.15
        self.label_height = 0.14
        self.predict_horizon, self.predict_step = 3.0, 0.1
        self.warning_time, self.min_predict_speed = 1.0, 0.1
        self.occupancy_filter = Occupancy(0.2, 0.5)
        self.tracker = BoxTracker(0.4, 0.5, 0.4, trail_length=60)
        self.box_tracks = []
        self.region, self.draft, self.obstacle_clusters = [], [], []
        self.editing = False
        self.last_valid_scan_time = None
        self.background = None
        self.prediction_enabled = True
        self.prediction = prediction
        self.state = 'NO_REGION'
        self.menu_notice = ''
        self.visualization = RegionVisualization(
            self.frame, self.label_height, lambda: self.get_clock().now().to_msg(),
            self.warning_time,
        )
        self.marker_pub = Publisher()
        self.region_pub = Publisher()
        self.state_pub = Publisher()
        self.flag_pub = Publisher()
        self.warning_pub = Publisher()

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=123)))

    def get_logger(self):
        return SimpleNamespace(info=lambda *a, **k: None, warning=lambda *a, **k: None,
                               error=lambda *a, **k: None)


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


class Recorder:
    def __init__(self, region_file):
        self.node = DetectorHarness(region_file)
        self.region_file = region_file
        self.snapshots = []

    def record(self, name, now):
        node = self.node
        with patch('gl5_detection.gl5_obstacle_node.time.monotonic', return_value=now):
            node.update_state_and_publish()
        self.snapshots.append({
            'name': name,
            'markers': node.marker_pub.message,
            'polygon': node.region_pub.message,
            'state': node.state_pub.message,
            'detected': node.flag_pub.message,
            'warning': node.warning_pub.message,
            'draft': node.draft.copy(),
            'saved': (json.loads(self.region_file.read_text())
                      if self.region_file.exists() else None),
        })

    def receive(self, ranges, now, frame='laser'):
        with patch('gl5_detection.gl5_obstacle_node.time.monotonic', return_value=now):
            self.node.scan_callback(scan(ranges, frame))

    def result(self):
        return json.loads(json.dumps(self.snapshots))


def replay(region_file):
    recorder = Recorder(region_file)
    node, record, receive = recorder.node, recorder.record, recorder.receive

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
    return recorder.result()


def replay_prediction(region_file):
    recorder = Recorder(region_file)
    node, record, receive = recorder.node, recorder.record, recorder.receive
    node.region = [(0.2, -0.5), (2.0, -0.5), (2.0, 0.5), (0.2, 0.5)]
    node.save()

    def approaching(distance):
        return [distance] * 5 + [math.inf] * 6

    def step(index, distance):
        now = 1.0 + index * 0.05
        receive(approaching(distance), now)
        return now

    now = step(0, 3.5)
    record('first sight, no speed yet', now)
    for index in range(1, 7):
        now = step(index, 3.5 - index * 0.05)
    record('tracked outside, not yet warning', now)
    for index in range(7, 23):
        now = step(index, 3.5 - index * 0.05)
    record('warning: predicted entry', now)
    for index in range(23, 32):
        now = step(index, 3.5 - index * 0.05)
    record('inside, entry pending', now)
    for index in range(32, 35):
        now = step(index, 3.5 - index * 0.05)
    record('occupied', now)
    for index in range(35, 42):
        now = step(index, 1.75 + (index - 34) * 0.05)
    record('leaving, exit pending', now)
    receive([math.inf] * 7 + [8.0] * 4, 3.6)
    record('gone, clear', 3.6)
    return recorder.result()


def load_or_update_fixture(name, actual):
    fixture = Path(__file__).parent / 'fixtures' / name
    if os.environ.get('UPDATE_FIXTURES'):
        fixture.write_text(json.dumps(actual, indent=2) + '\n')
    return json.loads(fixture.read_text())


class RegionAnchorTest(unittest.TestCase):
    def test_region_and_clicks_follow_background_pose(self):
        import numpy as np
        with tempfile.TemporaryDirectory() as directory:
            node = DetectorHarness(Path(directory) / 'region.json')
            yaw = math.pi / 2
            node.background = SimpleNamespace(
                ready=True, learning=False, aligned=True, fitness=1.0,
                pose=np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
                contour=lambda: [], foreground=lambda r, a, i: list(r),
            )
            node.region = [(1.0, -0.5), (2.0, -0.5), (2.0, 0.5), (1.0, 0.5)]
            laser = node.to_laser(node.region)
            for (x, y), (ex, ey) in zip(laser, [(-0.5, -1.0), (-0.5, -2.0), (0.5, -2.0), (0.5, -1.0)]):
                self.assertAlmostEqual(x, ex); self.assertAlmostEqual(y, ey)
            node.edit()
            point = PointStamped(); point.header.frame_id = 'laser'
            point.point.x, point.point.y = 0.0, -1.5
            with patch('gl5_detection.gl5_obstacle_node.time.monotonic', return_value=0.0):
                node.clicked_point_callback(point)
            self.assertAlmostEqual(node.draft[0][0], 1.5); self.assertAlmostEqual(node.draft[0][1], 0.0)
            node.cancel()
            with patch('gl5_detection.gl5_obstacle_node.time.monotonic', return_value=1.0):
                node.update_state_and_publish()
            polygon = node.region_pub.message['polygon']['points']
            self.assertAlmostEqual(polygon[0]['x'], -0.5); self.assertAlmostEqual(polygon[0]['y'], -1.0)


class ObstacleBehaviorTest(unittest.TestCase):
    def test_interactive_menu_matches_original(self):
        fixture = Path(__file__).parent / 'fixtures' / 'region_menu.json'
        self.assertEqual(
            message_to_ordereddict(make_region_menu('laser')),
            json.loads(fixture.read_text()),
        )

    def assert_matches_fixture(self, name, actual):
        expected = load_or_update_fixture(name, actual)
        self.assertEqual(len(actual), len(expected))
        for result, original in zip(actual, expected):
            with self.subTest(state=original['name']):
                self.assertEqual(result, original)

    def test_rviz_messages_and_state_match_original(self):
        with tempfile.TemporaryDirectory() as directory:
            actual = replay(Path(directory) / 'region.json')
        self.assert_matches_fixture('obstacle_behavior.json', actual)

    def test_prediction_scenario(self):
        with tempfile.TemporaryDirectory() as directory:
            actual = replay_prediction(Path(directory) / 'region.json')
        states = [(s['state']['data'], s['detected']['data'], s['warning']['data'])
                  for s in actual]
        self.assertEqual(states, [
            ('CLEAR', False, False), ('CLEAR', False, False), ('WARNING', False, True),
            ('WARNING', False, True), ('OCCUPIED', True, True), ('OCCUPIED', True, True),
            ('CLEAR', False, False),
        ])
        by_ns = lambda snapshot, ns: [m for m in snapshot['markers']['markers'] if m['ns'] == ns]
        warning = actual[2]
        self.assertEqual(len(by_ns(warning, 'predictions')), 1)
        self.assertEqual(len(by_ns(warning, 'entry_points')), 1)
        self.assertEqual(len(by_ns(warning, 'trails')), 1)
        self.assertIn('in', by_ns(warning, 'obstacle_labels')[0]['text'])
        self.assertAlmostEqual(by_ns(warning, 'entry_points')[0]['pose']['position']['x'], 2.0, 1)
        self.assertEqual(by_ns(actual[4], 'predictions'), [])
        self.assertEqual(by_ns(actual[5], 'predictions'), [])
        self.assert_matches_fixture('prediction_behavior.json', actual)

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
