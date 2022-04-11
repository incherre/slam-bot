'''This is the unit test file for the explorer.py file.'''

import unittest
from math import sqrt

import sim_framework
from explorer import SpaceStatus, Explorer
from collision_map import MapLocation
from slam import Slam

class MockCollisionMap:
    def __init__(self):
        pass

    def get_key(self, x, y):
        return (0, 0)

    def get_location(self, x, y):
        return MapLocation()

    def get_neighbor_keys(self, x, y):
        return []

class MockSlam(Slam):
    def __init__(self):
        self.map = MockCollisionMap()

    def get_estimated_position(self):
        return (0, 0, 0)

    def get_collision_map(self):
        return self.map

    def move_observe_and_update(self, theta, distance):
        pass

class TestExplorerMethods(unittest.TestCase):
    def setUp(self):
        self.mock_slam = MockSlam()
        self.explorer = Explorer(self.mock_slam)

    def tearDown(self):
        del self.mock_slam
        del self.explorer

    def test_init(self):
        self.assertIsInstance(self.explorer, Explorer)

        
if __name__ == '__main__':
    unittest.main()
