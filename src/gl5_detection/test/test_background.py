import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gl5_detection.detection_core import BackgroundModel


class BackgroundTest(unittest.TestCase):
    def learned(self, frames, **kwargs):
        model = BackgroundModel(learn_frames=len(frames), **kwargs)
        model.start_learning()
        finished = [model.observe(frame) for frame in frames]
        self.assertEqual(finished, [False] * (len(frames) - 1) + [True])
        self.assertTrue(model.ready)
        self.assertFalse(model.learning)
        return model

    def test_passthrough_until_ready(self):
        model = BackgroundModel(learn_frames=3)
        self.assertEqual(model.foreground([1.0, math.inf]), [1.0, math.inf])
        self.assertFalse(model.observe([1.0, 2.0]))  # not learning yet
        self.assertFalse(model.ready)

    def test_median_ignores_passing_object_and_keeps_open_beams_open(self):
        wall = [4.0, 4.0, 4.0, math.inf]
        frames = [wall, [1.0, 4.0, 4.0, math.inf], wall, [4.0, 0.0, 4.0, 2.0], wall]
        model = self.learned(frames)
        self.assertEqual(list(model.background), [4.0, 4.0, 4.0, math.inf])

    def test_foreground_threshold_uses_margin_and_ratio(self):
        model = self.learned([[10.0, 10.0, 10.0, math.inf]] * 2, margin=0.1, ratio=0.02)
        # threshold = 10 - (0.1 + 0.2) = 9.7
        result = model.foreground([9.5, 9.8, 10.5, 3.0])
        self.assertEqual(result, [9.5, math.inf, math.inf, 3.0])
        self.assertEqual(model.foreground([0.0, math.nan, -1.0, math.inf]), [math.inf] * 4)

    def test_beam_count_change_restarts_learning(self):
        model = self.learned([[4.0, 4.0]] * 2)
        self.assertEqual(model.foreground([1.0, 1.0, 1.0]), [1.0, 1.0, 1.0])
        self.assertTrue(model.learning)
        self.assertFalse(model.ready)
        self.assertFalse(model.observe([1.0, 1.0, 1.0]))
        self.assertTrue(model.observe([1.0, 1.0, 1.0]))
        self.assertEqual(list(model.background), [1.0, 1.0, 1.0])

    def test_contour_and_validation(self):
        model = self.learned([[2.0, math.inf, 2.0]] * 2)
        points = model.contour(0.0, math.pi / 2, stride=1)
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0][0], 2.0)
        self.assertAlmostEqual(points[1][0], -2.0)
        self.assertEqual(BackgroundModel(learn_frames=2).contour(0.0, 0.1), [])
        with self.assertRaises(ValueError):
            BackgroundModel(learn_frames=0)
        with self.assertRaises(ValueError):
            BackgroundModel(margin=-0.1)


if __name__ == '__main__':
    unittest.main()
