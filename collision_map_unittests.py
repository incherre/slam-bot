'''This is the unit test file for the collision_map.py file.'''

import unittest
from collision_map import MapLocation, CollisionMap
from math import radians, sqrt

class TestCollisionMap(unittest.TestCase):
    def setUp(self):
        self.empty_map = CollisionMap()

    def tearDown(self):
        del self.empty_map

    def test_init(self):
        self.assertIsInstance(self.empty_map, CollisionMap)
        self.assertEqual(self.empty_map.scale, 5)
        self.assertEqual(self.empty_map.max_dist, 100)
        self.assertEqual(self.empty_map.map, {})

    def test_flat_observation(self):
        self.empty_map.record_observations(0, 0, 0, [(0, 10)])

        expected_start = MapLocation()
        expected_start.stepped_count += 1
        
        expected_middle = MapLocation()
        expected_middle.missed_count += 1

        expected_end = MapLocation()
        expected_end.hit_count += 1

        self.assertEqual(set(self.empty_map.map.keys()), set([(0, 0), (5, 0), (10, 0)]))
        self.assertEqual(self.empty_map.get_location(0, 0), expected_start)
        self.assertEqual(self.empty_map.get_location(5, 0), expected_middle)
        self.assertEqual(self.empty_map.get_location(10, 0), expected_end)

    def test_angled_observation(self):
        self.empty_map.record_observations(0, 0, 0, [(radians(45), 10 * sqrt(2))])

        expected_start = MapLocation()
        expected_start.stepped_count += 1
        
        expected_middle = MapLocation()
        expected_middle.missed_count += 1

        expected_end = MapLocation()
        expected_end.hit_count += 1

        self.assertEqual(set(self.empty_map.map.keys()), set([(0, 0), (5, 5), (10, 10)]))
        self.assertEqual(self.empty_map.get_location(0, 0), expected_start)
        self.assertEqual(self.empty_map.get_location(5, 5), expected_middle)
        self.assertEqual(self.empty_map.get_location(10, 10), expected_end)

    def test_long_observation(self):
        self.empty_map.max_dist = 8
        self.empty_map.record_observations(0, 0, 0, [(0, 10)])

        expected_start = MapLocation()
        expected_start.stepped_count += 1
        
        expected_middle = MapLocation()
        expected_middle.missed_count += 1

        self.assertEqual(set(self.empty_map.map.keys()), set([(0, 0), (5, 0)]))
        self.assertEqual(self.empty_map.get_location(0, 0), expected_start)
        self.assertEqual(self.empty_map.get_location(5, 0), expected_middle)

    def test_complex_observation(self):
        self.empty_map.record_observations(0, 0, 0, [(0, 10), (radians(90), 10)])

        expected_start = MapLocation()
        expected_start.stepped_count += 1
        
        expected_middle = MapLocation()
        expected_middle.missed_count += 1

        expected_end = MapLocation()
        expected_end.hit_count += 1

        self.assertEqual(set(self.empty_map.map.keys()), set([(0, 0), (5, 0), (10, 0), (0, 5), (0, 10)]))
        self.assertEqual(self.empty_map.get_location(0, 0), expected_start)
        self.assertEqual(self.empty_map.get_location(5, 0), expected_middle)
        self.assertEqual(self.empty_map.get_location(10, 0), expected_end)
        self.assertEqual(self.empty_map.get_location(0, 5), expected_middle)
        self.assertEqual(self.empty_map.get_location(0, 10), expected_end)

    def test_multiple_observation(self):
        self.empty_map.record_observations(0, 5, 0, [(0, 10)])
        self.empty_map.record_observations(0, 0, 0, [(0, 10)])

        expected_start = MapLocation()
        expected_start.stepped_count += 1
        
        expected_middle = MapLocation()
        expected_middle.missed_count += 1

        expected_end = MapLocation()
        expected_end.hit_count += 1

        self.assertEqual(set(self.empty_map.map.keys()), set([(0, 0), (5, 0), (10, 0), (0, 5), (5, 5), (10, 5)]))
        self.assertEqual(self.empty_map.get_location(0, 0), expected_start)
        self.assertEqual(self.empty_map.get_location(5, 0), expected_middle)
        self.assertEqual(self.empty_map.get_location(10, 0), expected_end)
        self.assertEqual(self.empty_map.get_location(0, 5), expected_start)
        self.assertEqual(self.empty_map.get_location(5, 5), expected_middle)
        self.assertEqual(self.empty_map.get_location(10, 5), expected_end)

    def test_self_angled_observation(self):
        self.empty_map.record_observations(0, 0, radians(45), [(0, 10 * sqrt(2))])

        expected_start = MapLocation()
        expected_start.stepped_count += 1
        
        expected_middle = MapLocation()
        expected_middle.missed_count += 1

        expected_end = MapLocation()
        expected_end.hit_count += 1

        self.assertEqual(set(self.empty_map.map.keys()), set([(0, 0), (5, 5), (10, 10)]))
        self.assertEqual(self.empty_map.get_location(0, 0), expected_start)
        self.assertEqual(self.empty_map.get_location(5, 5), expected_middle)
        self.assertEqual(self.empty_map.get_location(10, 10), expected_end)

    def test_crossing_observation(self):
        self.empty_map.record_observations(0, 0, 0, [(0, 10)])
        self.empty_map.record_observations(5, -5, radians(90), [(0, 10)])

        expected_start = MapLocation()
        expected_start.stepped_count += 1
        
        expected_middle = MapLocation()
        expected_middle.missed_count += 2

        expected_end = MapLocation()
        expected_end.hit_count += 1

        self.assertEqual(set(self.empty_map.map.keys()), set([(0, 0), (5, 0), (10, 0), (5, -5), (5, 5)]))
        self.assertEqual(self.empty_map.get_location(0, 0), expected_start)
        self.assertEqual(self.empty_map.get_location(5, 0), expected_middle)
        self.assertEqual(self.empty_map.get_location(10, 0), expected_end)
        self.assertEqual(self.empty_map.get_location(5, -5), expected_start)
        self.assertEqual(self.empty_map.get_location(5, 5), expected_end)

    def test_rep(self):
        self.empty_map.record_observations(0, 0, 0, [])
        self.assertEqual(str(self.empty_map), "v1\nscale,max_dist\n5,100\nx,y,stepped_count,missed_count,hit_count\n0,0,1,0,0")

    def test_key_function(self):
        self.assertEqual(self.empty_map.get_key(0, 0), (0, 0))
        self.assertEqual(self.empty_map.get_key(2, -2), (0, 0))
        self.assertEqual(self.empty_map.get_key(3, 3), (5, 5))
        self.assertEqual(self.empty_map.get_key(2, 10), (0, 10))
        self.assertEqual(self.empty_map.get_key(-10, -2), (-10, 0))

    def test_map_deserialize(self):
        non_default_map = CollisionMap(collision_map_scale=50, collision_map_max_dist=1000)
        non_default_map.record_observations(0, 0, 0, [(0, 100)])
        serialized_map_string = str(non_default_map)
        deserialized_map = CollisionMap.from_string(serialized_map_string)

        expected_start = MapLocation()
        expected_start.stepped_count += 1
        
        expected_middle = MapLocation()
        expected_middle.missed_count += 1

        expected_end = MapLocation()
        expected_end.hit_count += 1

        self.assertEqual(deserialized_map.scale, 50)
        self.assertEqual(deserialized_map.max_dist, 1000)
        self.assertEqual(set(deserialized_map.map.keys()), set([(0, 0), (50, 0), (100, 0)]))
        self.assertEqual(deserialized_map.get_location(0, 0), expected_start)
        self.assertEqual(deserialized_map.get_location(50, 0), expected_middle)
        self.assertEqual(deserialized_map.get_location(100, 0), expected_end)

    def test_get_locations_within_rectangle_error_checking(self):
        # Duplicate points.
        self.assertRaises(AssertionError,
                          self.empty_map.get_locations_within_rectangle,
                          (0, 0), (0, 0), (1, 1), (2, 2))
        self.assertRaises(AssertionError,
                          self.empty_map.get_locations_within_rectangle,
                          (0, 0), (1, 1), (0, 0), (2, 2))
        self.assertRaises(AssertionError,
                          self.empty_map.get_locations_within_rectangle,
                          (0, 0), (1, 1), (2, 2), (0, 0))
        self.assertRaises(AssertionError,
                          self.empty_map.get_locations_within_rectangle,
                          (0, 0), (1, 1), (1, 1), (2, 2))
        self.assertRaises(AssertionError,
                          self.empty_map.get_locations_within_rectangle,
                          (0, 0), (1, 1), (2, 2), (1, 1))
        self.assertRaises(AssertionError,
                          self.empty_map.get_locations_within_rectangle,
                          (0, 0), (1, 1), (2, 2), (2, 2))

        # Not a rectangle.
        self.assertRaises(AssertionError,
                          self.empty_map.get_locations_within_rectangle,
                          (0, 0), (0, 1), (1, 1), (2, 2))

        # Wrong order.
        self.assertRaises(AssertionError,
                          self.empty_map.get_locations_within_rectangle,
                          (0, 0), (0, 1), (1, 0), (1, 1))

    def test_get_locations_within_rectangle(self):
        self.assertEqual(self.empty_map.get_locations_within_rectangle(
            (0, 0), (0, 1), (1, 1), (1, 0)), {})

        # Fill in a square.
        for i in range(0, 50, 5):
            self.empty_map.record_observations(0, i, 0, [(0, 50)])

        self.assertEqual(set(self.empty_map.get_locations_within_rectangle(
            (0, 0), (0, 10), (10, 10), (10, 0)).keys()),
                         set([(0, 0), (0, 5), (0, 10),
                              (5, 0), (5, 5), (5, 10),
                              (10, 0), (10, 5), (10, 10)]))

        self.assertEqual(set(self.empty_map.get_locations_within_rectangle(
            (1, 5), (5, 9), (9, 5), (5, 1)).keys()),
                         set([(0, 5), (5, 0),
                              (5, 5), (5, 10),
                              (10, 5)]))

        self.assertEqual(set(self.empty_map.get_locations_within_rectangle(
            (0, -10), (0, 10), (10, 10), (10, -10)).keys()),
                         set([(0, 0), (0, 5), (0, 10),
                              (5, 0), (5, 5), (5, 10),
                              (10, 0), (10, 5), (10, 10)]))

        self.assertEqual(set(self.empty_map.get_locations_within_rectangle(
            (1, 5), (10, 14), (14, 10), (5, 1)).keys()),
                         set([(0, 5), (5, 0),
                              (5, 5), (5, 10),
                              (10, 5), (10, 10),
                              (10, 15), (15, 10)]))

if __name__ == '__main__':
    unittest.main()
