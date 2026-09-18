import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import math
from gl5_detection.detection_core import BoxTracker, cluster_scan, predict_entry, classify_tracks


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

    def test_trail_follows_centres_and_is_snapshotted(self):
        tracker=BoxTracker(trail_length=3)
        for step in range(5):
            last=tracker.update([box(step*0.1)],step*0.025)[0]
        self.assertEqual([round(x,2) for x,_ in last.trail],[0.2,0.3,0.4])
        tracker.update([box(0.5)],0.125)
        self.assertEqual(len(last.trail),3)


SQUARE=[(1.0,-0.5),(2.0,-0.5),(2.0,0.5),(1.0,0.5)]


class PredictionTest(unittest.TestCase):
    def test_straight_approach_enters_at_edge(self):
        t,point=predict_entry((3.0,0.0),(-1.0,0.0),SQUARE,horizon=3.0,step=0.1)
        self.assertAlmostEqual(t,1.0)
        self.assertAlmostEqual(point[0],2.0)

    def test_no_prediction_when_slow_inside_receding_or_too_far(self):
        self.assertIsNone(predict_entry((3.0,0.0),(-0.05,0.0),SQUARE,min_speed=0.1))
        self.assertIsNone(predict_entry((1.5,0.0),(-1.0,0.0),SQUARE))
        self.assertIsNone(predict_entry((3.0,0.0),(1.0,0.0),SQUARE))
        self.assertIsNone(predict_entry((6.0,0.0),(-1.0,0.0),SQUARE,horizon=3.0))

    def test_classify_tracks_sets_region_flags(self):
        tracker=BoxTracker()
        for step in range(10):
            tracks=tracker.update([box(3.0-step*0.05),box(1.5)],step*0.025)
        outside,inside=classify_tracks(tracks,SQUARE)
        self.assertFalse(outside.in_region); self.assertTrue(inside.in_region)
        self.assertIsNotNone(outside.time_to_enter); self.assertIsNone(inside.time_to_enter)

    def test_cluster_whole_scan_without_polygon(self):
        ranges=[1.5]*5+[math.inf]+[5.0]*5
        self.assertEqual(len(cluster_scan(ranges,-0.05,0.01,0.0,60.0,None,min_points=3)),2)
        self.assertEqual(len(cluster_scan(ranges,-0.05,0.01,0.0,60.0,SQUARE,min_points=3)),1)


if __name__ == '__main__':
    unittest.main()
