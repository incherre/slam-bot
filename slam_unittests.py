'''This is the unit test file for the slam.py file.'''

import unittest
from math import sqrt, radians, degrees

import sim_framework
from slam import *
from collision_map import MapLocation

def point_distance(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

class TestSlamMethods(unittest.TestCase):
    def setUp(self):
        self.world = sim_framework.World()

        # Walls for RANSAC.
        self.world.add_obs(sim_framework.Wall(-15, '-x'))
        self.world.add_obs(sim_framework.Wall(15, '+x'))
        self.world.add_obs(sim_framework.Wall(-15, '-y'))
        self.world.add_obs(sim_framework.Wall(15, '+y'))

        # Small box for Spikes.
        self.world.add_obs(sim_framework.Box(3, 3.2, -3.2, -3))

        self.bot = sim_framework.CircleBot(1, 0, 0, 0)
        self.world.add_ent(self.bot)

        self.bot_control = sim_framework.SimBotControl(self.world, self.bot)

        options = {}
        options[SPIKE_THRESHOLD] = 12

        options[RANSAC_MAX_TRIES] = 20
        options[RANSAC_SAMPLES] = 10
        options[RANSAC_RANGE] = radians(135)
        options[RANSAC_ERROR] = 0.02
        options[RANSAC_CONSENSUS] = 90

        options[EKF_INITIAL_UNCERTAINTY] = 0.0001
        options[EKF_ODOMETRY_NOISE] = 0.01
        options[EKF_RANGE_NOISE] = 0.02
        options[EKF_BEARING_NOISE] = radians(1)
        options[EKF_INNOVATION_LAMBDA] = 1
        options[EKF_LANDMARK_THRESHOLD] = 1

        options[COLLISION_MAP_SCALE] = 5
        options[COLLISION_MAP_MAX_DISTANCE] = 30
        self.slam = Slam(self.bot_control, option_dictionary = options)

    def tearDown(self):
        del self.slam
        del self.bot_control
        del self.bot
        del self.world

    def test_init(self):
        self.assertIsInstance(self.world, sim_framework.World)
        self.assertIsInstance(self.bot, sim_framework.CircleBot)
        self.assertIsInstance(self.bot_control, sim_framework.SimBotControl)
        self.assertIsInstance(self.slam, Slam)

    def test_position_getter(self):
        self.assertEqual(self.slam.get_estimated_position(), (0, 0, 0))

    def test_move_observe_and_update(self):
        self.slam.move_observe_and_update(radians(90), 5)

        x, y, theta = self.slam.get_estimated_position()
        self.assertAlmostEqual(x, 0)
        self.assertAlmostEqual(y, 5)
        self.assertAlmostEqual(theta, radians(90))

        self.slam.move_observe_and_update(radians(90), 5)
        self.slam.move_observe_and_update(radians(90), 5)

        x, y, theta = self.slam.get_estimated_position()
        # Should be -5, 0, radians(270).
        self.assertTrue(abs(x + 5) < 0.1)
        self.assertTrue(abs(y - 0) < 0.1)
        self.assertTrue(abs(degrees(theta) - 270) < 1)

    def test_get_collision_map(self):
        self.slam.move_observe_and_update(0, 0)

        expected_start = MapLocation()
        expected_start.stepped_count += 1

        # Many beams pass through the same grid square
        expected_middle = MapLocation()
        expected_middle.missed_count += 29

        # Many beams hit in the same grid square
        expected_end = MapLocation()
        expected_end.hit_count += 19

        self.assertEqual(self.slam.get_collision_map().get_location(0, 0), expected_start)
        self.assertEqual(self.slam.get_collision_map().get_location(-10, 0), expected_middle)
        self.assertEqual(self.slam.get_collision_map().get_location(-15, 0), expected_end)
        
if __name__ == '__main__':
    unittest.main()
