#!/usr/bin/env python3
"""Use the already-running real /scan. Does not publish scan data or change saved user regions."""
import json
import math
from pathlib import Path
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray


def main():
    rclpy.init()
    node = Node('roi_live_verifier')
    data = {'scan_count': 0, 'state': None, 'boxes': 0}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def scan(msg):
        data['scan_count'] += 1
        data['samples'] = len(msg.ranges)
        data['valid'] = sum(math.isfinite(x) and x > 0 for x in msg.ranges)
    node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)
    node.create_subscription(String, '/gl5/obstacle_state', lambda m: data.update(state=m.data), qos)
    node.create_subscription(MarkerArray, '/gl5/obstacle_markers', lambda m: data.update(
        boxes=sum(x.ns == 'obstacles' and x.action == 0 for x in m.markers)), qos)
    def pump(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)
    pump(1)
    if any(name == 'gl5_obstacle_detector' for name, _ in node.get_node_names_and_namespaces()):
        raise SystemExit('Stop the obstacle detector before this test; keep the GL5 driver running')
    root = Path(__file__).resolve().parents[1]
    executable = root / 'install/gl5_driver/lib/gl5_driver/gl5_obstacle_node.py'
    with tempfile.TemporaryDirectory(prefix='gl5-live-roi-') as directory:
        filename = Path(directory) / 'region.json'
        proc = subprocess.Popen([str(executable), '--ros-args', '-p', f'region_file:={filename}'])
        try:
            client = node.create_client(Trigger, '/gl5/region/load')
            assert client.wait_for_service(timeout_sec=5), 'Detector did not start'
            def load(vertices):
                filename.write_text(json.dumps({'version':1, 'frame_id':'laser', 'vertices':vertices}))
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                assert future.done() and future.result().success, 'Region load failed'
                pump(2)
            # Broad diagnostic region encloses measured returns; not the user's final region.
            load([[-61,-61],[61,-61],[61,61],[-61,61]])
            occupied = data.copy()
            assert occupied['state'] == 'OCCUPIED' and occupied['boxes'] > 0, occupied
            # Diagnostic region outside the instrument's measurement range contains no returns.
            load([[70,70],[71,70],[71,71],[70,71]])
            clear = data.copy()
            assert clear['state'] == 'CLEAR' and clear['boxes'] == 0, clear
            assert clear['scan_count'] > occupied['scan_count'] > 0
            report = {'passed':True, 'source':'live GL5 /scan', 'occupied_region':occupied, 'empty_region':clear}
            (root / 'artifacts/roi_live_verification.json').write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(report, indent=2))
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
