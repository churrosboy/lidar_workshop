#!/usr/bin/env python3
"""Synthetic ROS integration test. Use a separate ROS_DOMAIN_ID from the live sensor."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray, Marker, InteractiveMarkerFeedback
from visualization_msgs.srv import GetInteractiveMarkers


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '91':
        raise SystemExit('Run with ROS_DOMAIN_ID=91 to isolate synthetic scans from the live sensor')
    rclpy.init()
    node = Node('roi_integration_test')
    state = {'value': None, 'flag': None, 'boxes': 0, 'labels': []}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/gl5/obstacle_state', lambda m: state.update(value=m.data), qos)
    node.create_subscription(Bool, '/gl5/obstacle_detected', lambda m: state.update(flag=m.data), qos)
    node.create_subscription(MarkerArray, '/gl5/obstacle_markers', lambda m: state.update(
        boxes=sum(x.ns == 'obstacles' and x.action == Marker.ADD for x in m.markers),
        labels=[x for x in m.markers if x.ns == 'obstacle_labels' and x.action == Marker.ADD]), qos)
    scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    click_pub = node.create_publisher(PointStamped, '/clicked_point', 10)
    menu_pub = node.create_publisher(InteractiveMarkerFeedback, '/gl5/region_menu/feedback', 10)
    def pump(seconds, distance=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if distance is not None:
                scan = LaserScan()
                scan.header.frame_id = 'laser'
                scan.header.stamp = node.get_clock().now().to_msg()
                scan.angle_min, scan.angle_increment = -0.05, 0.01
                scan.range_min, scan.range_max = 0.0, 60.0
                scan.ranges = [float(distance)] * 11
                scan_pub.publish(scan)
            rclpy.spin_once(node, timeout_sec=0.025)
            time.sleep(0.01)
    def service(name, expected=True):
        client = node.create_client(Trigger, '/gl5/region/' + name)
        if not client.wait_for_service(timeout_sec=5):
            raise AssertionError(f'Service missing: {name}')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.done() and future.result().success == expected, name
        node.destroy_client(client)
        pump(0.15)
    def click(x, y):
        point = PointStamped()
        point.header.frame_id = 'laser'
        point.point.x, point.point.y = float(x), float(y)
        click_pub.publish(point)
        pump(0.15)
    def expect(value, flag=False, boxes=None):
        assert state['value'] == value and state['flag'] == flag, state
        if boxes is not None:
            assert state['boxes'] == boxes, state
            assert len(state['labels']) == boxes
    executable = Path(__file__).resolve().parents[1] / 'install/gl5_driver/lib/gl5_driver/gl5_obstacle_node.py'
    with tempfile.TemporaryDirectory(prefix='gl5-roi-test-') as directory:
        region_file = Path(directory) / 'region.json'
        params = Path(__file__).resolve().parents[1] / 'src/gl5_driver/config/obstacles.yaml'
        proc = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(params),
                                 '-p', f'region_file:={region_file}'])
        try:
            service('edit')
            expect('EDITING')
            # Too few vertices must fail without applying a region.
            click(0.5,-0.3)
            service('finish', False)
            click(2,-0.3)
            click(2,0.3)
            click(0.5,0.3)
            click(0.5,-0.3)  # Close by clicking the first vertex.
            expect('NO_DATA')
            assert len(json.loads(region_file.read_text())['vertices']) == 4
            pump(0.5, 3)
            expect('CLEAR', boxes=0)
            pump(0.4, 1)
            expect('OCCUPIED', True, 1)
            label = state['labels'][0]
            assert label.type == Marker.TEXT_VIEW_FACING and label.color.r == 1.0
            assert label.color.g < 0.2 and 0 < label.scale.z < 0.2
            assert 'x' in label.text and 'm/s' in label.text and '--' not in label.text
            assert ' ' not in label.text and len(label.text.splitlines()) == 3
            pump(0.15, 3)
            expect('OCCUPIED', True)
            pump(0.6, 3)
            expect('CLEAR', boxes=0)
            pump(0.4, 1)
            pump(1.3)
            expect('NO_DATA', boxes=0)
            pump(0.15, 1)
            expect('CLEAR')  # Recovered data must earn the enter delay again.
            pump(0.3, 1)
            expect('OCCUPIED', True)
            pump(0.2, math.inf)
            expect('NO_DATA', boxes=0)
            service('edit')
            click(0,0)
            service('undo')
            service('cancel')
            pump(0.4, 1)
            expect('OCCUPIED', True)
            service('load')
            pump(0.4, 1)
            expect('OCCUPIED', True)
            menu_client = node.create_client(GetInteractiveMarkers, '/gl5/region_menu/get_interactive_markers')
            assert menu_client.wait_for_service(timeout_sec=5)
            future = menu_client.call_async(GetInteractiveMarkers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.done() and future.result().markers
            menu = next(m for m in future.result().markers if m.name == 'region_menu')
            clear_id = next(m.id for m in menu.menu_entries if 'Clear region' in m.title)
            feedback = InteractiveMarkerFeedback()
            feedback.header.frame_id = 'laser'
            feedback.client_id = 'roi_test'
            feedback.marker_name = menu.name
            feedback.control_name = 'menu'
            feedback.event_type = InteractiveMarkerFeedback.MENU_SELECT
            feedback.menu_entry_id = clear_id
            menu_pub.publish(feedback)
            pump(0.4)
            expect('NO_REGION', boxes=0)
            assert json.loads(region_file.read_text())['vertices'] == []
            print('PASS: clicks, polygon commit/save/load, clustering, temporal gate, NO_DATA, RViz menu clear and marker cleanup')
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
