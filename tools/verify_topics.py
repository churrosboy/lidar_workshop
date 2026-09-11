#!/usr/bin/env python3
"""Observe real /scan and /points, check conversion, and save evidence (no publisher)."""
import argparse
import collections
import json
import math
import struct
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2


class Observer(Node):
    def __init__(self):
        super().__init__('gl5_verifier')
        self.scans = collections.OrderedDict()
        self.clouds = collections.OrderedDict()
        self.counts = {'scan': 0, 'points': 0, 'matched': 0, 'valid_ranges': 0}
        self.errors = []
        self.arrivals = {'scan': [], 'points': []}
        self.latest = None
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/points', self.cloud, qos_profile_sensor_data)

    def check(self, condition, message):
        if not condition and message not in self.errors:
            self.errors.append(message)

    def receive(self, kind, msg, cache):
        self.counts[kind] += 1
        self.arrivals[kind].append(time.monotonic())
        self.check(msg.header.frame_id == 'laser', f'{kind}: unexpected frame_id')
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.check(key != (0, 0), f'{kind}: zero timestamp')
        cache[key] = msg
        while len(cache) > 256:
            cache.popitem(last=False)
        return key

    def scan(self, msg):
        key = self.receive('scan', msg, self.scans)
        self.latest = msg
        n = len(msg.ranges)
        self.check(n >= 2 and n == len(msg.intensities), 'scan: array size mismatch')
        self.check(abs(msg.angle_max - msg.angle_min - (n - 1) * msg.angle_increment) < 1e-4,
                   'scan: angular extent mismatch')
        self.check(msg.angle_increment > 0 and abs((msg.angle_max - msg.angle_min) - 1.5 * math.pi) < 1e-4,
                   'scan: unexpected field of view')
        self.check(0 <= msg.scan_time < 5 and msg.time_increment == 0, 'scan: invalid timing')
        for distance in msg.ranges:
            self.check((math.isfinite(distance) and msg.range_min <= distance <= msg.range_max)
                       or distance == math.inf, 'scan: invalid range encoding')
        self.counts['valid_ranges'] += sum(math.isfinite(r) for r in msg.ranges)
        self.match(key)

    def cloud(self, msg):
        key = self.receive('points', msg, self.clouds)
        self.check(msg.height == 1 and msg.point_step == 16 and msg.row_step == msg.width * 16,
                   'points: invalid dimensions')
        self.check(len(msg.data) == msg.row_step * msg.height, 'points: buffer size mismatch')
        self.check([(f.name, f.offset, f.datatype, f.count) for f in msg.fields] ==
                   [('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1), ('intensity', 12, 7, 1)],
                   'points: unexpected fields')
        self.match(key)

    def match(self, key):
        if key not in self.scans or key not in self.clouds:
            return
        scan, cloud = self.scans.pop(key), self.clouds.pop(key)
        self.check(cloud.width == len(scan.ranges), 'paired sample count mismatch')
        if cloud.point_step != 16 or len(cloud.data) != len(scan.ranges) * 16:
            return
        endian = '>' if cloud.is_bigendian else '<'
        for i, distance in enumerate(scan.ranges):
            x, y, z, intensity = struct.unpack_from(endian + 'ffff', cloud.data, i * 16)
            if math.isfinite(distance):
                angle = scan.angle_min + i * scan.angle_increment
                self.check(abs(x - distance * math.cos(angle)) < 1e-4 and
                           abs(y - distance * math.sin(angle)) < 1e-4 and z == 0,
                           'paired scan/cloud coordinate mismatch')
            else:
                self.check(math.isnan(x) and math.isnan(y) and math.isnan(z), 'invalid point is not NaN')
            if i < len(scan.intensities):
                self.check(intensity == scan.intensities[i], 'paired intensity mismatch')
        self.counts['matched'] += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--output', default='artifacts/ros_verification.json')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be positive')
    rclpy.init()
    node = Observer()
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
        for kind, times in node.arrivals.items():
            node.check(len(times) >= 10, f'{kind}: fewer than 10 messages')
            node.check(bool(times) and times[0] - start < 10, f'{kind}: no data in first 10 seconds')
            node.check(bool(times) and time.monotonic() - times[-1] < 5, f'{kind}: stream stalled')
            node.check(not times or max([b - a for a, b in zip(times, times[1:])] or [0]) < 5,
                       f'{kind}: gap exceeded 5 seconds')
        node.check(node.counts['matched'] >= 10, 'fewer than 10 matched scan/cloud pairs')
        node.check(node.counts['valid_ranges'] > 0, 'no finite ranges')
        frequencies = {kind: (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0
                       for kind, times in node.arrivals.items()}
        report = {'passed': not node.errors, 'elapsed_seconds': time.monotonic() - start,
                  'counts': node.counts, 'average_hz': frequencies, 'errors': node.errors,
                  'timestamp_source': 'PC receive time'}
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + '\n')
        if node.latest:
            scan = node.latest
            snapshot = {'frame_id': scan.header.frame_id, 'angle_min': scan.angle_min,
                        'angle_increment': scan.angle_increment,
                        'ranges_m': [float(r) if math.isfinite(r) else None for r in scan.ranges],
                        'intensities': list(scan.intensities)}
            output.with_name(output.stem + '_scan.json').write_text(json.dumps(snapshot, indent=2) + '\n')
        print(json.dumps(report, indent=2))
        return 0 if report['passed'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
