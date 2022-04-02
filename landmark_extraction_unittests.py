'''This is the unit test file for the landmark_extraction.py file.'''

import unittest
from landmark_extraction import normalize_angle, normalized_angle_difference, extract_spike, extract_ransac
from math import cos, sin, radians

class TestSupportingFunctions(unittest.TestCase):
    def test_normalize_angle(self):
        self.assertAlmostEqual(normalize_angle(radians(-180)), radians(180))
        self.assertAlmostEqual(normalize_angle(radians(-540)), radians(180))
        self.assertAlmostEqual(normalize_angle(radians(540)), radians(180))
        self.assertAlmostEqual(normalize_angle(radians(900)), radians(180))

    def test_normalized_angle_difference(self):
        self.assertAlmostEqual(normalized_angle_difference(radians(0), radians(180)), radians(180))
        self.assertAlmostEqual(normalized_angle_difference(radians(0), radians(190)), radians(-170))
        self.assertAlmostEqual(normalized_angle_difference(radians(190), radians(0)), radians(170))
        self.assertAlmostEqual(normalized_angle_difference(radians(350), radians(10)), radians(20))
        self.assertAlmostEqual(normalized_angle_difference(radians(10), radians(350)), radians(-20))

class TestExtractSpike(unittest.TestCase):
    def test_simple_extraction(self):
        pos = (0, 0, 0)
        range_scan = [(radians(0),10), (radians(1),5), (radians(2),10)]

        expected_landmarks = [(5 * cos(radians(1)), 5 * sin(radians(1)))]

        self.assertEqual(extract_spike(pos, range_scan), expected_landmarks)

    def test_simple_extraction_not_at_origin(self):
        pos = (2, 5, radians(73))
        range_scan = [(radians(0),10), (radians(1),5), (radians(2),10)]

        expected_landmarks = [(2 + (5 * cos(radians(74))), 5 + (5 * sin(radians(74))))]

        self.assertEqual(extract_spike(pos, range_scan), expected_landmarks)

    def test_skip_too_close(self):
        pos = (0, 0, 0)
        range_scan = [(radians(0),10), (radians(1),-5), (radians(2),10)]

        expected_landmarks = []

        self.assertEqual(extract_spike(pos, range_scan), expected_landmarks)

    def test_no_wrap_around(self):
        pos = (0, 0, 0)
        range_scan = [(radians(0),10), (radians(1),10), (radians(2),5)]

        expected_landmarks = []

        self.assertEqual(extract_spike(pos, range_scan), expected_landmarks)

    def test_wrap_around(self):
        pos = (0, 0, 0)
        range_scan = [(radians(0),5), (radians(90),10), (radians(180),10), (radians(270),10)]

        expected_landmarks = [(5 * cos(radians(0)), 5 * sin(radians(0)))]

        self.assertEqual(extract_spike(pos, range_scan), expected_landmarks)

class TestExtractRANSAC(unittest.TestCase):
    def test_circle_scan(self):
        pos = (0, 0, 0)
        range_scan = [(radians(i), 10) for i in range(360)]

        expected_landmarks = []

        self.assertEqual(extract_ransac(pos, range_scan, seed_override=0), expected_landmarks)

    def test_square_scan(self):
        pos = (0, 0, 0)
        range_scan = [(radians(i + 0.5), min(10 / abs(sin(radians(i + 0.5))), 10 / abs(cos(radians(i + 0.5))))) for i in range(360)]

        expected_landmarks = [(-10, 0), (10, 0), (0, -10), (0, 10)]
        actual_landmarks = extract_ransac(pos, range_scan, seed_override=0)

        self.assertEqual(len(expected_landmarks), len(actual_landmarks))
        for i in range(len(expected_landmarks)):
            self.assertEqual(len(actual_landmarks[i]), 2)
            self.assertAlmostEqual(expected_landmarks[i][0], actual_landmarks[i][0])
            self.assertAlmostEqual(expected_landmarks[i][1], actual_landmarks[i][1])

    def test_square_scan_with_rotation(self):
        pos = (0, 0, radians(45))
        range_scan = [(radians(i + 0.5), min(10 / abs(sin(radians(i + 0.5))), 10 / abs(cos(radians(i + 0.5))))) for i in range(360)]

        coord = 10 * cos(radians(45))
        expected_landmarks = [(-coord, -coord), (coord, coord),
                              (coord, -coord), (-coord, coord)]
        actual_landmarks = extract_ransac(pos, range_scan, seed_override=0)

        self.assertEqual(len(expected_landmarks), len(actual_landmarks))
        for i in range(len(expected_landmarks)):
            self.assertEqual(len(actual_landmarks[i]), 2)
            self.assertAlmostEqual(expected_landmarks[i][0], actual_landmarks[i][0])
            self.assertAlmostEqual(expected_landmarks[i][1], actual_landmarks[i][1])

    def test_square_scan_with_translation(self):
        pos = (2, 5, 0)
        range_scan = [(radians(i + 0.5), min(10 / abs(sin(radians(i + 0.5))), 10 / abs(cos(radians(i + 0.5))))) for i in range(360)]

        expected_landmarks = [(-8, 0), (12, 0), (0, -5), (0, 15)]
        actual_landmarks = extract_ransac(pos, range_scan, seed_override=0)

        self.assertEqual(len(expected_landmarks), len(actual_landmarks))
        for i in range(len(expected_landmarks)):
            self.assertEqual(len(actual_landmarks[i]), 2)
            self.assertAlmostEqual(expected_landmarks[i][0], actual_landmarks[i][0])
            self.assertAlmostEqual(expected_landmarks[i][1], actual_landmarks[i][1])

if __name__ == '__main__':
    unittest.main()
