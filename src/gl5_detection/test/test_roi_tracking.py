import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gl5_detection.detection_core import BoxTracker


def box(x,y=0):
    return [(x-0.2,y-0.1),(x+0.2,y+0.1)]


class TrackingTest(unittest.TestCase):
    def test_known_linear_speed_and_dimensions(self):
        tracker = BoxTracker()
        first = tracker.update([box(0)], 0)[0]
        self.assertIsNone(first.speed)
        for step in range(1,21):
            last = tracker.update([box(step*0.05)], step*0.025)[0]
        self.assertEqual(first.id, last.id)
        self.assertAlmostEqual(last.speed, 2.0)
        x0,y0,x1,y1=last.box
        self.assertAlmostEqual(x1-x0,0.4)
        self.assertAlmostEqual(y1-y0,0.2)

    def test_stationary(self):
        tracker=BoxTracker()
        for step in range(20):
            last=tracker.update([box(1)],step*0.025)[0]
        self.assertAlmostEqual(last.speed,0)

    def test_order_changes_do_not_exchange_ids(self):
        tracker=BoxTracker()
        a,b=tracker.update([box(0),box(2)],0)
        b2,a2=tracker.update([box(2.01),box(0.01)],0.025)
        self.assertEqual(a.id,a2.id)
        self.assertEqual(b.id,b2.id)

    def test_expiry_reset_and_disappearance(self):
        tracker=BoxTracker()
        first=tracker.update([box(0)],0)[0]
        self.assertEqual(tracker.update([],0.1),[])
        later=tracker.update([box(0)],0.7)[0]
        self.assertNotEqual(first.id,later.id)
        self.assertIsNone(later.speed)
        tracker.reset()
        self.assertIsNone(tracker.update([box(0)],0.8)[0].speed)

    def test_short_occlusion_restarts_speed_estimate(self):
        tracker=BoxTracker()
        for step in range(10):
            last=tracker.update([box(step*0.01)],step*0.025)[0]
        self.assertIsNotNone(last.speed)
        returned=tracker.update([box(0.2)],0.5)[0]
        self.assertIsNone(returned.speed)


if __name__ == '__main__':
    unittest.main()
