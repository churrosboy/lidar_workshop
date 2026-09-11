import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from roi_geometry import validate_polygon, inside, clusters, bounds, Occupancy


class PolygonTest(unittest.TestCase):
    def test_concave_and_boundary(self):
        poly = validate_polygon([(0,0),(2,0),(2,1),(1,1),(1,2),(0,2)])
        self.assertTrue(inside((0.5,1.5), poly))
        self.assertTrue(inside((1,1), poly))
        self.assertFalse(inside((1.5,1.5), poly))
        self.assertEqual(inside((0.5,1.5), poly), inside((0.5,1.5), list(reversed(poly))))

    def test_invalid_regions(self):
        for vertices in ([], [(0,0),(1,1),(2,2)], [(0,0),(2,2),(0,2),(2,0)],
                         [(0,0),(1,0),(0,0)], [(0,0),(1,0),(0,float('nan'))],
                         [(0,0),(True,1),(1,0)]):
            with self.subTest(vertices=vertices), self.assertRaises(ValueError):
                validate_polygon(vertices)

    def test_saved_integer_coordinates_become_ros_floats(self):
        polygon = validate_polygon([(0,0),(1,0),(0,1)])
        self.assertTrue(all(isinstance(v, float) for p in polygon for v in p))

    def test_clusters_exclude_outside_and_isolated_returns(self):
        poly = validate_polygon([(0.2,-0.5),(2,-0.5),(2,0.5),(0.2,0.5)])
        self.assertEqual(clusters([3.0]*7, -0.03, 0.01, 0, 60, poly), [])
        self.assertEqual(clusters([1.0, math.inf, 1.0], -0.01, 0.01, 0, 60, poly), [])
        groups = clusters([1.0]*5 + [math.inf] + [1.5]*5, -0.05, 0.01, 0, 60, poly)
        self.assertEqual([len(g) for g in groups], [5,5])
        self.assertGreater(bounds(groups[1])[0], bounds(groups[0])[2])

    def test_distance_jump_breaks_cluster(self):
        poly = validate_polygon([(-3,-3),(3,-3),(3,3),(-3,3)])
        self.assertEqual(len(clusters([1.0]*5 + [2.0]*5, 0, 0.001, 0, 60, poly)), 2)

    def test_box_is_not_a_square(self):
        self.assertEqual(bounds([(1,0),(2,0.1),(1.5,0.3)]), (1,0,2,0.3))

    def test_enter_exit_delays(self):
        gate = Occupancy(0.2,0.5)
        self.assertFalse(gate.update(True, 0))
        self.assertFalse(gate.update(True, 0.1))
        self.assertTrue(gate.update(True, 0.21))
        self.assertTrue(gate.update(False, 0.3))
        self.assertTrue(gate.update(False, 0.7))
        self.assertFalse(gate.update(False, 0.81))

    def test_interruption_resets_candidate(self):
        gate = Occupancy()
        gate.update(True, 0)
        gate.update(False, 0.1)
        self.assertFalse(gate.update(True, 0.21))
        gate.reset()
        self.assertFalse(gate.update(True, 1.0))


if __name__ == '__main__':
    unittest.main()
