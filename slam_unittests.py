'''This is the unit test file for the slam.py file.'''

import unittest
import sim_framework
from slam import LandmarkType, Landmark, LandmarkDB, Slam
from math import sqrt

def point_distance(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

class TestLandmarkDBMethods(unittest.TestCase):
    def setUp(self):
        self.empty_db = LandmarkDB()
        self.populated_db = LandmarkDB()
        self.populated_db.insert_landmark(Landmark(LandmarkType.SPIKE, 1, 0))
        self.populated_db.insert_landmark(Landmark(LandmarkType.SPIKE, 10, 0))
        self.populated_db.insert_landmark(Landmark(LandmarkType.RANSAC, 0, 1))
        self.populated_db.insert_landmark(Landmark(LandmarkType.RANSAC, 0, 10))

    def tearDown(self):
        del self.empty_db
        del self.populated_db

    def test_init(self):
        self.assertIsInstance(self.empty_db, LandmarkDB)
        self.assertIsInstance(self.populated_db, LandmarkDB)

    def test_insert(self):
        insert_db = LandmarkDB()
        landmark = Landmark(LandmarkType.SPIKE, 0, 0)
        insert_db.insert_landmark(landmark)
        self.assertEqual(insert_db.find_nearest(LandmarkType.SPIKE, 0, 0), landmark)

    def test_find(self):
        self.assertIsNone(self.empty_db.find_nearest(LandmarkType.SPIKE, 0, 0))
        self.assertIsNone(self.empty_db.find_nearest(LandmarkType.RANSAC, 0, 0))

        spike_near_0_0 = self.populated_db.find_nearest(LandmarkType.SPIKE, 0, 0)
        self.assertIsNotNone(spike_near_0_0)
        self.assertEqual(spike_near_0_0, Landmark(LandmarkType.SPIKE, 1, 0))
        self.assertEqual(spike_near_0_0.times_seen, 1)

        spike_near_20_0 = self.populated_db.find_nearest(LandmarkType.SPIKE, 20, 0)
        self.assertIsNotNone(spike_near_20_0)
        self.assertEqual(spike_near_20_0, Landmark(LandmarkType.SPIKE, 10, 0))
        self.assertEqual(spike_near_20_0.times_seen, 1)

        ransac_near_0_0 = self.populated_db.find_nearest(LandmarkType.RANSAC, 0, 0)
        self.assertIsNotNone(ransac_near_0_0)
        self.assertEqual(ransac_near_0_0, Landmark(LandmarkType.RANSAC, 0, 1))
        self.assertEqual(ransac_near_0_0.times_seen, 1)

        ransac_near_0_20 = self.populated_db.find_nearest(LandmarkType.RANSAC, 0, 20)
        self.assertIsNotNone(ransac_near_0_20)
        self.assertEqual(ransac_near_0_20, Landmark(LandmarkType.RANSAC, 0, 10))
        self.assertEqual(ransac_near_0_20.times_seen, 1)

    def test_update(self):
        update_db = LandmarkDB()
        update_db.insert_landmark(Landmark(LandmarkType.SPIKE, 0, 0))

        update_db.find_nearest(LandmarkType.SPIKE, 0, 0).increment_seen()
        self.assertEqual(update_db.find_nearest(LandmarkType.SPIKE, 0, 0).times_seen, 2)

        update_db.find_nearest(LandmarkType.SPIKE, 0, 0).update_postition(1, 1)
        self.assertEqual(update_db.find_nearest(LandmarkType.SPIKE, 0, 0), Landmark(LandmarkType.SPIKE, 1, 1))

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

    def test_landmark_extraction(self):
        spikes = self.slam.extract_spike(self.slam.control.get_distance_reading())
        self.assertEqual(len(spikes), 2)
        self.assertTrue(min(map(lambda p: point_distance(p, (3.1, -3)), spikes)) < 0.01)
        self.assertTrue(min(map(lambda p: point_distance(p, (3, -3.1)), spikes)) < 0.01)

        ransacs = self.slam.extract_ransac(self.slam.control.get_distance_reading(), seed_override=0)
        self.assertEqual(len(ransacs), 4)
        self.assertTrue(min(map(lambda p: point_distance(p, (15, 0)), ransacs)) < 1)
        self.assertTrue(min(map(lambda p: point_distance(p, (-15, 0)), ransacs)) < 1)
        self.assertTrue(min(map(lambda p: point_distance(p, (0, 15)), ransacs)) < 0.1)
        self.assertTrue(min(map(lambda p: point_distance(p, (0, -15)), ransacs)) < 0.1)

if __name__ == '__main__':
    unittest.main()
