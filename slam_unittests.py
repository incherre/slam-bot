'''This is the unit test file for the slam.py file.'''

import unittest
import sim_framework
from slam import LandmarkType, Slam
from math import sqrt

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

        self.slam = Slam(self.bot_control)

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
        
if __name__ == '__main__':
    unittest.main()
