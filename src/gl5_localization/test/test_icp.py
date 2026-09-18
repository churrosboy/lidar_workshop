import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from gl5_localization.icp import Target, apply, icp, make_transform, scan_to_points, to_pose


def room(step=0.005):
    """Walls of a 6 x 4 m room plus a pillar, so the outline has no rotational symmetry."""
    xs, ys = np.arange(-3, 3, step), np.arange(-2, 2, step)
    walls = [np.column_stack((xs, np.full_like(xs, -2.0))),
             np.column_stack((xs, np.full_like(xs, 2.0))),
             np.column_stack((np.full_like(ys, -3.0), ys)),
             np.column_stack((np.full_like(ys, 3.0), ys))]
    angles = np.arange(0, 2 * math.pi, 0.02)
    pillar = np.column_stack((1.5 + 0.3 * np.cos(angles), 0.8 + 0.3 * np.sin(angles)))
    return np.vstack(walls + [pillar])


ROOM = room()


def simulated_scan(pose, beams=1500, fov_deg=270.0, stride=3):
    """What a GL5 at `pose` (odom -> sensor) would see: nearest surface per beam."""
    local = apply(np.linalg.inv(pose), ROOM)
    angles = np.degrees(np.arctan2(local[:, 1], local[:, 0]))
    distances = np.hypot(local[:, 0], local[:, 1])
    bins = np.floor((angles + fov_deg / 2) / fov_deg * beams).astype(int)
    ranges = np.full(beams, np.inf)
    for beam, distance in zip(bins, distances):
        if 0 <= beam < beams:
            ranges[beam] = min(ranges[beam], distance)
    return scan_to_points(ranges, math.radians(-fov_deg / 2),
                          math.radians(fov_deg) / (beams - 1), stride=stride)


class IcpTest(unittest.TestCase):
    def assert_recovers(self, motion, init=None, tol_m=0.01, tol_deg=0.5):
        before, after = simulated_scan(np.eye(3)), simulated_scan(motion)
        estimate, fitness = icp(after, before, init=init)
        x, y, yaw = to_pose(estimate)
        ex, ey, eyaw = to_pose(motion)
        self.assertLess(math.hypot(x - ex, y - ey), tol_m)
        self.assertLess(abs(math.degrees(yaw - eyaw)), tol_deg)
        self.assertGreater(fitness, 0.8)

    def test_small_motion_without_initial_guess(self):
        self.assert_recovers(make_transform(0.10, 0.05, math.radians(3)))
        self.assert_recovers(make_transform(-0.15, 0.0, math.radians(-8)))

    def test_large_rotation_recovers_with_a_guess(self):
        motion = make_transform(0.3, -0.2, math.radians(60))
        self.assert_recovers(motion, init=make_transform(0.25, -0.15, math.radians(55)))

    def test_initial_guess_improves_large_rotation(self):
        motion = make_transform(0.3, -0.2, math.radians(60))
        before, after = simulated_scan(np.eye(3)), simulated_scan(motion)
        blind, _ = icp(after, before)
        guided, _ = icp(after, before, init=make_transform(0.25, -0.15, math.radians(55)))
        blind_error = abs(math.degrees(to_pose(blind)[2]) - 60)
        guided_error = abs(math.degrees(to_pose(guided)[2]) - 60)
        self.assertLess(guided_error, blind_error / 10)

    def test_target_normals_follow_walls(self):
        target = Target(simulated_scan(np.eye(3)))
        # Away from the corners, where the neighbourhood spans two walls.
        bottom_wall = (np.abs(target.points[:, 1] + 2.0) < 0.02) & (np.abs(target.points[:, 0]) < 2.5)
        self.assertTrue(bottom_wall.any())
        self.assertTrue(np.allclose(np.abs(target.normals[bottom_wall][:, 1]), 1.0, atol=1e-3))

    def test_no_overlap_reports_low_fitness(self):
        before = simulated_scan(np.eye(3))
        _, fitness = icp(before + np.array([50.0, 50.0]), before)
        self.assertLess(fitness, 0.1)
        _, fitness = icp(before[:3], before)  # fewer than min_pairs
        self.assertEqual(fitness, 0.0)

    def test_scan_to_points_filters_and_strides(self):
        ranges = [math.inf, 1.0, 0.05, 2.0, 40.0, 3.0, math.nan, 4.0]
        points = scan_to_points(ranges, 0.0, math.pi / 2, min_range=0.1, max_range=30.0, stride=2)
        # valid beams 1, 3, 5, 7 -> stride 2 keeps beams 1 (90 deg) and 5 (450 deg = 90 deg)
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0][1], 1.0)
        self.assertAlmostEqual(points[1][1], 3.0)

    def test_pose_roundtrip(self):
        x, y, yaw = to_pose(make_transform(1.0, -2.0, 0.7))
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, -2.0)
        self.assertAlmostEqual(yaw, 0.7)


if __name__ == '__main__':
    unittest.main()
