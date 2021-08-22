'''This is the unit test file for the slam.py file.'''

import unittest
import numpy as np
from math import radians, sin, cos
from ekf import EKF

class TestEKF(unittest.TestCase):
    def setUp(self):
        self.empty_ekf = EKF(landmark_threshold=0)

    def tearDown(self):
        del self.empty_ekf

    def test_init(self):
        self.assertIsInstance(self.empty_ekf, EKF)
        self.assertEqual(self.empty_ekf.landmark_types, [])
        self.assertEqual(self.empty_ekf.landmark_counts, [])

        np.testing.assert_array_equal(
            self.empty_ekf.system_state,
            np.array([[0], [0], [0]]))
        np.testing.assert_array_equal(
            self.empty_ekf.covariance,
            np.array([[0.95, 0, 0],
                      [0, 0.95, 0],
                      [0, 0, 0.95]]))

    def test_noop_move(self):
        # Run the move function, but without actually moving.
        self.empty_ekf.update(radians(0), 0, [])

        np.testing.assert_array_equal(
            self.empty_ekf.pos(),
            np.array([[0], [0], [0]]))
        np.testing.assert_array_equal(
            self.empty_ekf.covariance,
            np.array([[0.95, 0, 0],
                      [0, 0.95, 0],
                      [0, 0, 0.95]]))

    def test_simple_move(self):
        # Turn left 30 degrees and advance 10 units of distance.
        self.empty_ekf.update(radians(30), 10, [])

        np.testing.assert_almost_equal(
            self.empty_ekf.pos(),
            np.array([[8.66025404], [5], [0.52359878]]))
        np.testing.assert_almost_equal(
            self.empty_ekf.covariance,
            np.array([[75.95, -38.9711432, -8.0005164],
                      [-38.9711432, 25.95, 4.8808997],
                      [-8.0005164, 4.8808997, 0.9637078]]))

    def test_linear_move_and_observe(self):
        # Add a landmark at the start.
        self.empty_ekf.update(radians(0), 0, [(20, 0, 'statue')])

        np.testing.assert_array_equal(
            self.empty_ekf.pos(),
            np.array([[0], [0], [0]]))
        np.testing.assert_array_equal(
            self.empty_ekf.system_state,
            np.array([[0], [0], [0], [20], [0]]))
        np.testing.assert_array_equal(
            self.empty_ekf.covariance,
            np.array([[0.95, 0, 0, 0.95, 0],
                      [0, 0.95, 0, 0, 0.95],
                      [0, 0, 0.95, 0, 0],
                      [0.95, 0, 0, 1.15, 0],
                      [0, 0.95, 0, 0, 0.95]]))

        # Move 5 units, but imply some error by landmark drift.
        self.empty_ekf.update(radians(0), 5, [(19, 0, 'statue')])

        np.testing.assert_almost_equal(
            self.empty_ekf.pos(),
            np.array([[5.8274554], [0], [-0.042244909]]))
        np.testing.assert_almost_equal(
            self.empty_ekf.system_state,
            np.array([[5.8274554], [0], [-0.042244909], [19.9014031], [0]]))
        np.testing.assert_almost_equal(
            self.empty_ekf.covariance,
            np.array([[25.95, 0, -4.75, 0.95, 0],
                      [0, 0.95, 0, 0, 0.95],
                      [-4.75, 0, 0.95, 0, 0],
                      [0.95, 0, 0, 1.15, 0],
                      [0, 0.95, 0, 0, 0.95]]))

    def test_rotate_and_observe(self):
        # Add a landmark at the start.
        self.empty_ekf.update(radians(0), 0, [(20, 0, 'statue')])

        np.testing.assert_array_equal(
            self.empty_ekf.pos(),
            np.array([[0], [0], [0]]))
        np.testing.assert_array_equal(
            self.empty_ekf.system_state,
            np.array([[0], [0], [0], [20], [0]]))
        np.testing.assert_array_equal(
            self.empty_ekf.covariance,
            np.array([[0.95, 0, 0, 0.95, 0],
                      [0, 0.95, 0, 0, 0.95],
                      [0, 0, 0.95, 0, 0],
                      [0.95, 0, 0, 1.15, 0],
                      [0, 0.95, 0, 0, 0.95]]))

        # Turn 30 degrees, but imply some error by landmark drift.
        self.empty_ekf.update(radians(30), 0, [(20 * cos(radians(-2)), 20 * sin(radians(-2)), 'statue')])

        np.testing.assert_almost_equal(
            self.empty_ekf.pos(),
            np.array([[0], [0], [0.55788443]]))
        np.testing.assert_almost_equal(
            self.empty_ekf.system_state,
            np.array([[0], [0], [0.55788443], [20], [0]]))
        np.testing.assert_almost_equal(
            self.empty_ekf.covariance,
            np.array([[0.95, 0, 0, 0.95, 0],
                      [0, 0.95, 0, 0, 0.95],
                      [0, 0, 0.9637078, 0, 0],
                      [0.95, 0, 0, 1.15, 0],
                      [0, 0.95, 0, 0, 0.95]]))

    def test_add_two_landmarks(self):
        # Add first landmark.
        self.empty_ekf.update(radians(0), 0, [(20, 0, 'statue')])

        np.testing.assert_array_equal(
            self.empty_ekf.pos(),
            np.array([[0], [0], [0]]))
        np.testing.assert_array_equal(
            self.empty_ekf.system_state,
            np.array([[0], [0], [0], [20], [0]]))
        np.testing.assert_array_equal(
            self.empty_ekf.covariance,
            np.array([[0.95, 0, 0, 0.95, 0],
                      [0, 0.95, 0, 0, 0.95],
                      [0, 0, 0.95, 0, 0],
                      [0.95, 0, 0, 1.15, 0],
                      [0, 0.95, 0, 0, 0.95]]))

        # Add second landmark.
        self.empty_ekf.update(radians(0), 0, [(0, 20, 'fountain')])

        np.testing.assert_array_equal(
            self.empty_ekf.pos(),
            np.array([[0], [0], [0]]))
        np.testing.assert_array_equal(
            self.empty_ekf.system_state,
            np.array([[0], [0], [0], [20], [0], [0], [20]]))
        np.testing.assert_array_equal(
            self.empty_ekf.covariance,
            np.array([[0.95, 0, 0, 0.95, 0, 0.95, 0],
                      [0, 0.95, 0, 0, 0.95, 0, 0.95],
                      [0, 0, 0.95, 0, 0, 0, 0],
                      [0.95, 0, 0, 1.15, 0, 0.95, 0],
                      [0, 0.95, 0, 0, 0.95, 0, 0.95],
                      [0.95, 0, 0, 0.95, 0, 1.15, 0],
                      [0, 0.95, 0, 0, 0.95, 0, 0.95]]))

if __name__ == '__main__':
    unittest.main()
