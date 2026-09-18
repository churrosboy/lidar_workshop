#!/usr/bin/env python3
"""Keyframe ICP scan matcher: odometry, path and odom->laser TF from /scan alone.

Each scan is aligned to the current keyframe (not the previous scan) so drift only
accumulates when the keyframe changes. The previous motion seeds the next alignment.
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from gl5_localization import icp as matching


class ScanMatcherNode(Node):
    def __init__(self):
        super().__init__('gl5_scan_matcher')
        self.configure_parameters()
        self.keyframe = None            # matching.Target built from the keyframe scan
        self.odom_from_keyframe = np.eye(3)
        self.keyframe_from_sensor = np.eye(3)
        self.last_motion = np.eye(3)    # sensor motion between the last two scans
        self.last_stamp = None
        self.scan_count = 0
        self.match_time_total = 0.0
        self.match_time_count = 0
        self.path = Path()
        self.path.header.frame_id = self.odom_frame
        self.odom_pub = self.create_publisher(Odometry, '/gl5/odom', 10)
        self.path_pub = self.create_publisher(Path, '/gl5/path', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'Scan matcher: {self.odom_frame} -> {self.base_frame}, stride {self.stride}, '
            f'keyframe every {self.keyframe_distance} m / {math.degrees(self.keyframe_angle):.0f} deg')

    def configure_parameters(self) -> None:
        descriptor = ParameterDescriptor(read_only=True)

        def param(name, default):
            return self.declare_parameter(name, default, descriptor).value

        self.odom_frame = param('odom_frame', 'odom')
        self.base_frame = param('base_frame', 'laser')
        self.publish_tf = param('publish_tf', True)
        self.stride = param('stride', 3)
        self.iterations = param('iterations', 20)
        self.max_correspondence = param('max_correspondence', 0.3)
        self.keyframe_distance = param('keyframe_distance', 0.2)
        self.keyframe_angle = math.radians(param('keyframe_angle_deg', 10.0))
        self.min_range = param('min_range', 0.1)
        self.max_range = param('max_range', 30.0)
        self.min_fitness = param('min_fitness', 0.5)
        self.min_points = param('min_points', 50)
        # Motion between two processed scans beyond this is a mismatch, not movement.
        self.max_step = param('max_step', 0.15)
        self.max_turn = math.radians(param('max_turn_deg', 15.0))
        self.process_every = param('process_every', 1)
        self.path_max_poses = param('path_max_poses', 2000)
        positive = (self.stride, self.iterations, self.max_correspondence, self.keyframe_distance,
                    self.keyframe_angle, self.max_range, self.process_every, self.path_max_poses,
                    self.min_points, self.max_step, self.max_turn)
        if (not self.odom_frame or not self.base_frame or self.odom_frame == self.base_frame
                or any(not math.isfinite(v) or v <= 0 for v in positive)
                or self.min_range < 0 or self.min_range >= self.max_range
                or not 0 < self.min_fitness <= 1):
            raise ValueError('Invalid scan matcher parameters')

    def scan_callback(self, msg: LaserScan) -> None:
        self.scan_count += 1
        if self.scan_count % self.process_every:
            return
        if msg.header.frame_id != self.base_frame:
            self.get_logger().warning(
                f'Scan frame {msg.header.frame_id!r} != base_frame {self.base_frame!r}',
                throttle_duration_sec=5.0)
            return
        points = matching.scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                         self.min_range, self.max_range, self.stride)
        if len(points) < self.min_points:
            self.get_logger().warning(f'Only {len(points)} usable returns; skipping scan',
                                      throttle_duration_sec=5.0)
            return
        if self.keyframe is None:
            self.set_keyframe(points, np.eye(3))
            self.publish(msg.header.stamp)
            return

        started = time.perf_counter()
        guess = self.keyframe_from_sensor @ self.last_motion
        transform, fitness = matching.icp(points, self.keyframe, guess, self.iterations,
                                          self.max_correspondence)
        self.match_time_total += time.perf_counter() - started
        self.match_time_count += 1
        motion = np.linalg.inv(self.keyframe_from_sensor) @ transform
        mx, my, myaw = matching.to_pose(motion)
        if fitness < self.min_fitness:
            reason = f'fitness {fitness:.2f} < {self.min_fitness}'
        elif (not np.all(np.isfinite(transform)) or math.hypot(mx, my) > self.max_step
              or abs(myaw) > self.max_turn):
            reason = f'implausible motion {math.hypot(mx, my):.2f} m / {math.degrees(myaw):.0f} deg'
        else:
            reason = None
        if reason:
            self.get_logger().warning(f'Match rejected ({reason}); keeping last pose',
                                      throttle_duration_sec=2.0)
            self.last_motion = np.eye(3)
            self.publish(msg.header.stamp)
            return

        self.last_motion = motion
        self.keyframe_from_sensor = transform
        x, y, yaw = matching.to_pose(transform)
        if math.hypot(x, y) > self.keyframe_distance or abs(yaw) > self.keyframe_angle:
            self.set_keyframe(points, self.odom_from_keyframe @ transform)
        self.publish(msg.header.stamp)
        self.get_logger().info(
            f'ICP {1e3 * self.match_time_total / self.match_time_count:.1f} ms avg, '
            f'fitness {fitness:.2f}, pose ({self.pose_text()})', throttle_duration_sec=5.0)

    def set_keyframe(self, points: np.ndarray, odom_from_keyframe: np.ndarray) -> None:
        self.keyframe = matching.Target(points)
        self.odom_from_keyframe = odom_from_keyframe
        self.keyframe_from_sensor = np.eye(3)

    def pose_text(self) -> str:
        x, y, yaw = matching.to_pose(self.odom_from_keyframe @ self.keyframe_from_sensor)
        return f'{x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg'

    def reset_odometry(self, why: str) -> None:
        self.get_logger().error(f'Odometry reset: {why}')
        self.keyframe = None
        self.odom_from_keyframe = np.eye(3)
        self.keyframe_from_sensor = np.eye(3)
        self.last_motion = np.eye(3)
        self.path.poses.clear()

    def publish(self, stamp) -> None:
        pose = self.odom_from_keyframe @ self.keyframe_from_sensor
        if not np.all(np.isfinite(pose)) or np.abs(pose[:2, 2]).max() > 1e4:
            self.reset_odometry('pose is not finite or beyond 10 km')
            pose = np.eye(3)
        x, y, yaw = matching.to_pose(pose)
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)

        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        dt = self.seconds(stamp) - self.last_stamp if self.last_stamp is not None else 0.0
        if dt > 0:  # velocity in the sensor frame, from the motion since the last scan
            mx, my, myaw = matching.to_pose(self.last_motion)
            odom.twist.twist.linear.x, odom.twist.twist.linear.y = mx / dt, my / dt
            odom.twist.twist.angular.z = myaw / dt
        self.last_stamp = self.seconds(stamp)
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        del self.path.poses[:-self.path_max_poses]
        self.path_pub.publish(self.path)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x, tf.transform.translation.y = x, y
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self.tf_broadcaster.sendTransform(tf)

    @staticmethod
    def seconds(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9


def main():
    rclpy.init()
    node = None
    try:
        node = ScanMatcherNode()
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
