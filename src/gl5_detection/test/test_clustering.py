import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gl5_detection.detection_core import cluster_scan

SQUARE = [(1.0, -0.5), (2.0, -0.5), (2.0, 0.5), (1.0, 0.5)]
ANGLE_MIN, INC = -0.05, 0.01


def clusters(ranges, polygon=None, min_points=3, max_gap=0.15):
    return cluster_scan(ranges, ANGLE_MIN, INC, 0.0, 60.0, polygon, min_points, max_gap)


class ClusteringTest(unittest.TestCase):
    def test_gap_of_invalid_beams_splits_two_objects(self):
        ranges = [1.5] * 5 + [math.inf] + [5.0] * 5
        groups = clusters(ranges)
        self.assertEqual(len(groups), 2)
        self.assertEqual([len(g) for g in groups], [5, 5])
        self.assertAlmostEqual(groups[0][0][0], 1.5 * math.cos(ANGLE_MIN))

    def test_polygon_keeps_only_inside_points(self):
        ranges = [1.5] * 5 + [math.inf] + [5.0] * 5
        self.assertEqual(len(clusters(ranges, SQUARE)), 1)
        self.assertEqual(len(clusters([5.0] * 11, SQUARE)), 0)

    def test_min_points_drops_small_groups(self):
        ranges = [1.5] * 2 + [math.inf] + [5.0] * 5
        self.assertEqual(len(clusters(ranges, min_points=3)), 1)
        self.assertEqual(len(clusters(ranges, min_points=2)), 2)

    def test_distance_jump_splits_neighbours(self):
        ranges = [1.0] * 4 + [1.5] * 4
        self.assertEqual(len(clusters(ranges, max_gap=0.15)), 2)
        self.assertEqual(len(clusters(ranges, max_gap=0.6)), 1)

    def test_invalid_and_out_of_range_beams_are_skipped(self):
        ranges = [0.0, math.nan, 70.0, 1.0, 1.0, 1.0, -1.0]
        groups = clusters(ranges)
        self.assertEqual(len(groups), 1)
        self.assertEqual(len(groups[0]), 3)
        self.assertEqual(clusters([math.inf] * 5), [])


if __name__ == '__main__':
    unittest.main()
