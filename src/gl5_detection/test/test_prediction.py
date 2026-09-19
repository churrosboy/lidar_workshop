import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gl5_detection.detection_core import BoxTracker
from gl5_detection.prediction import predict_entry, classify_tracks


def box(x, y=0):
    return [(x - 0.2, y - 0.1), (x + 0.2, y + 0.1)]


SQUARE = [(1.0, -0.5), (2.0, -0.5), (2.0, 0.5), (1.0, 0.5)]


class PredictionTest(unittest.TestCase):
    def test_straight_approach_enters_at_edge(self):
        t, point = predict_entry((3.0, 0.0), (-1.0, 0.0), SQUARE, horizon=3.0, step=0.1)
        self.assertAlmostEqual(t, 1.0)
        self.assertAlmostEqual(point[0], 2.0)

    def test_diagonal_approach(self):
        t, point = predict_entry((3.0, 1.0), (-1.0, -0.5), SQUARE, horizon=3.0, step=0.1)
        self.assertAlmostEqual(t, 1.0)
        self.assertAlmostEqual(point[0], 2.0)
        self.assertAlmostEqual(point[1], 0.5)

    def test_no_prediction_when_slow_inside_receding_or_too_far(self):
        self.assertIsNone(predict_entry((3.0, 0.0), (-0.05, 0.0), SQUARE, min_speed=0.1))
        self.assertIsNone(predict_entry((1.5, 0.0), (-1.0, 0.0), SQUARE))
        self.assertIsNone(predict_entry((3.0, 0.0), (1.0, 0.0), SQUARE))
        self.assertIsNone(predict_entry((6.0, 0.0), (-1.0, 0.0), SQUARE, horizon=3.0))

    def test_passing_by_never_enters(self):
        self.assertIsNone(predict_entry((3.0, 1.0), (-1.0, 0.0), SQUARE, horizon=5.0))

    def test_classify_tracks_sets_region_flags(self):
        tracker = BoxTracker()
        for step in range(10):
            tracks = tracker.update([box(3.0 - step * 0.05), box(1.5)], step * 0.025)
        outside, inside = classify_tracks(tracks, SQUARE)
        self.assertFalse(outside.in_region)
        self.assertTrue(inside.in_region)
        self.assertIsNotNone(outside.time_to_enter)
        self.assertIsNone(inside.time_to_enter)


if __name__ == '__main__':
    unittest.main()
