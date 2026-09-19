"""4단계 채점: ICP가 배경 지도 위에서 센서 움직임을 따라가는지 봅니다.

배경 차분(2단계)은 매 스캔을 배경 지도에 정렬한 뒤 지도에서 떨어진 점만 남깁니다.
그 정렬을 실제로 해 주는 것이 4단계의 icp() 입니다. 4단계를 구현하기 전에는
background.py 의 _align() 이 "센서가 고정돼 있다"고 가정하고 넘어가므로,
여기 있는 테스트는 4단계를 채운 뒤에야 통과합니다.

2단계 채점은 test_background.py 입니다.
"""
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'gl5_localization'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
from gl5_localization.icp import make_transform
from test_background import ANGLE_MIN, ANGLE_INC, foreground_count, learned, room_scan


class BackgroundAlignmentTest(unittest.TestCase):
    def test_small_sensor_rotation_is_absorbed_and_objects_still_detected(self):
        model = learned()
        turned = make_transform(0.05, -0.03, math.radians(8))
        self.assertEqual(foreground_count(model, room_scan(turned)), 0)
        self.assertTrue(model.aligned)
        self.assertAlmostEqual(math.degrees(math.atan2(model.pose[1, 0], model.pose[0, 0])), 8, delta=0.5)
        self.assertGreater(foreground_count(model, room_scan(turned, extra=[(2.0, -1.0, 0.2)])), 10)
        self.assertLess(foreground_count(model, room_scan(turned)), 3)

    def test_large_jump_freezes_pose_and_flags_misalignment(self):
        model = learned()
        moved = make_transform(1.5, 1.0, math.radians(60))
        model.foreground(room_scan(moved), ANGLE_MIN, ANGLE_INC)
        self.assertFalse(model.aligned)
        self.assertTrue(np.allclose(model.pose, np.eye(3)))


if __name__ == '__main__':
    unittest.main()
