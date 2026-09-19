import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'gl5_localization'))
import numpy as np
from gl5_detection.background import BackgroundModel
from gl5_localization.icp import apply, make_transform

BEAMS, FOV = 1000, 180.0
ANGLE_MIN, ANGLE_INC = math.radians(-FOV / 2), math.radians(FOV) / (BEAMS - 1)


def room_scan(pose=np.eye(3), extra=(), pillar=True):
    step = 0.005
    xs, ys = np.arange(-3, 3, step), np.arange(-2, 2, step)
    walls = [np.column_stack((xs, np.full_like(xs, -2.0))), np.column_stack((xs, np.full_like(xs, 2.0))),
             np.column_stack((np.full_like(ys, -3.0), ys)), np.column_stack((np.full_like(ys, 3.0), ys))]
    a = np.arange(0, 2 * math.pi, 0.02)
    surfaces = walls + ([np.column_stack((1.5 + 0.3 * np.cos(a), 0.8 + 0.3 * np.sin(a)))] if pillar else [])
    for cx, cy, radius in extra:
        surfaces.append(np.column_stack((cx + radius * np.cos(a), cy + radius * np.sin(a))))
    local = apply(np.linalg.inv(pose), np.vstack(surfaces))
    angles = np.degrees(np.arctan2(local[:, 1], local[:, 0]))
    bins = np.floor((angles + FOV / 2) / FOV * BEAMS).astype(int)
    ok = (bins >= 0) & (bins < BEAMS)
    ranges = np.full(BEAMS, np.inf)
    np.minimum.at(ranges, bins[ok], np.hypot(local[ok, 0], local[ok, 1]))
    return ranges.tolist()


def learned(frames=None, **kwargs):
    frames = frames or [room_scan()] * 3
    model = BackgroundModel(learn_frames=len(frames), **kwargs)
    model.start_learning()
    done = [model.observe(f, ANGLE_MIN, ANGLE_INC) for f in frames]
    assert done == [False] * (len(frames) - 1) + [True]
    return model


def foreground_count(model, ranges):
    return sum(math.isfinite(v) for v in model.foreground(ranges, ANGLE_MIN, ANGLE_INC))


class BackgroundTest(unittest.TestCase):
    def test_passthrough_until_ready(self):
        model = BackgroundModel(learn_frames=3)
        self.assertEqual(model.foreground([1.0, math.inf], 0.0, 0.1), [1.0, math.inf])
        self.assertFalse(model.observe([1.0, 2.0], 0.0, 0.1))
        self.assertFalse(model.ready)

    def test_static_scene_is_all_background(self):
        model = learned()
        self.assertEqual(foreground_count(model, room_scan()), 0)
        self.assertTrue(model.aligned)
        self.assertGreater(len(model.contour()), 50)

    def test_median_ignores_person_passing_during_learning(self):
        with_person = room_scan(extra=[(1.0, 0.0, 0.2)])
        model = learned([room_scan(), with_person, room_scan(), room_scan(), with_person])
        self.assertEqual(foreground_count(model, room_scan()), 0)
        self.assertGreater(foreground_count(model, with_person), 10)

    def test_new_object_and_removed_object_are_both_foreground(self):
        model = learned()
        self.assertGreater(foreground_count(model, room_scan(extra=[(2.0, -1.0, 0.2)])), 10)
        self.assertGreater(foreground_count(learned(), room_scan(pillar=False)), 5)

    def test_rotated_pose_keeps_scan_on_the_map(self):
        # 센서가 조금 돌아간 상황입니다. 포즈를 따라가는 일은 4단계 ICP 몫이라
        # 여기서는 포즈를 직접 넣어 두고, 그 포즈에서 지도 대조와 시야 가장자리
        # 처리가 맞는지만 봅니다. (포즈 추적 자체는 test_background_alignment.py)
        model = learned()
        turned = make_transform(0.05, -0.03, math.radians(8))
        model.pose = turned
        self.assertEqual(foreground_count(model, room_scan(turned)), 0)
        self.assertGreater(foreground_count(model, room_scan(turned, extra=[(2.0, -1.0, 0.2)])), 10)
        self.assertLess(foreground_count(model, room_scan(turned)), 3)

    def test_validation(self):
        with self.assertRaises(ValueError):
            BackgroundModel(learn_frames=0)
        with self.assertRaises(ValueError):
            BackgroundModel(margin=-0.1)


if __name__ == '__main__':
    unittest.main()
