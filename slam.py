'''An implementation of SLAM techniques.'''
from abc import ABC, abstractmethod
from math import radians, sin, cos, fsum, inf, sqrt
from random import choice, sample, seed
from enum import Enum, auto, unique

class SensingAndControl(ABC):
    '''The abstract class for obtaining readings and controlling the robot.'''

    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def get_distance_reading(self):
        '''Should return a list of readings one degree apart,
           going CW, starting in the forward direction.'''
        pass

    @abstractmethod
    def move(self, theta, distance):
        '''Turn theta then go distance and return odemetry.'''
        pass

def linear_regression(points):
    '''Calculates ax + by + c = 0 from the given points.'''

    n = len(points)
    sx = fsum([p[0] for p in points])
    sy = fsum([p[1] for p in points])
    sxx = fsum([p[0] ** 2 for p in points])
    sxy = fsum([p[0] * p[1] for p in points])
    syy = fsum([p[1] ** 2 for p in points])

    try:
        a = ((n * sxy) - (sx * sy)) / ((n * sxx) - (sx ** 2))
    except ZeroDivisionError:
        # if the line is vertical, y is not considered
        a = -1
        b = 0
        c = sx / n
    else:
        b = -1
        c = (sy / n) - ((a * sx) / n)

    return (a, b, c)

def min_distance(line, point):
    '''Calculates the minimum distance between a line and a point.'''

    a, b, c = line

    # extracting coordinates this way allows extra elements to be ignored
    x = point[0]
    y = point[1]

    return abs((a * x) + (b * y) + c) / sqrt((a ** 2) + (b ** 2))

def closest_point(line, point):
    '''Calculates the point on a line closest to another point.'''

    a, b, c = line

    # extracting coordinates this way allows extra elements to be ignored
    x = point[0]
    y = point[1]

    line_x = ((b * ((b * x) - (a * y))) - (a * c)) / ((a ** 2) + (b ** 2))
    line_y = ((a * ((a * y) - (b * x))) - (b * c)) / ((a ** 2) + (b ** 2))

    return (line_x, line_y)

@unique
class LandmarkType(Enum):
    SPIKE = auto()
    RANSAC = auto()

class Landmark:
    '''A representation of a SLAM landmark.'''

    def __init__(self, landmark_type, landmark_x, landmark_y):
        self.landmark_type = landmark_type
        self.x = landmark_x
        self.y = landmark_y
        self.times_seen = 1

    def increment_seen(self):
        self.times_seen += 1

    def update_postition(self, new_x, new_y):
        self.x = new_x
        self.y = new_y

    def __repr__(self):
        return "Landmark of type '{}', located at ({},{}), seen {} time/s.".format(
            self.landmark_type, self.x, self.y, self.times_seen)

    def __eq__(self, other):
        return isinstance(other, Landmark) and \
            self.landmark_type == other.landmark_type and \
            self.x == other.x and self.y == other.y

class LandmarkDB:
    '''Manages a database of landmarks.'''

    def __init__(self):
        self.db = []

    def find_nearest(self, landmark_type, landmark_x, landmark_y):
        '''Returns the nearest landmark of the same type. May return None.'''
        # TODO(Daniel): if this ends up being too slow, look into http://en.wikipedia.org/wiki/Quadtree
        # or maybe just hand tuned 2d buckets.

        nearest = None
        nearest_distance = 0
        for landmark in self.db:
            if landmark.landmark_type != landmark_type:
                continue

            this_distance = sqrt((landmark_x - landmark.x) ** 2 + (landmark_y - landmark.y) ** 2)
            
            if nearest is None or this_distance < nearest_distance:
                nearest = landmark
                nearest_distance = this_distance

        return nearest

    def insert_landmark(self, landmark):
        self.db.append(landmark)
                

class Slam:
    '''Manages a Slam instance.'''

    spike_threshold = 0.5

    ransac_max_tries = 1000
    ransac_samples = 5
    ransac_range = 20
    ransac_error = 0.5
    ransac_consensus = 30

    landmark_association_threshold = 5

    def __init__(self, control):
        self.control = control
        self.landmarks = LandmarkDB()
        self.pos = (0, 0, 0) # (x, y, theta)
        self.map = []

    def get_estimated_position(self):
        '''Returns the current estimate of the bot's position.'''
        return self.pos

    def move_observe_and_update(self, theta, distance):
        '''Turn theta then go distance, make a new observation, and update estimated location.'''
        # Issue commands to robot and recieve instrument readings.
        odemetry = self.control.move(theta, distance)
        observations = self.control.get_distance_reading()

        # Update belief.
        pos_x, pos_y, pos_theta = self.pos
        pos_theta += theta
        pos_x += odemetry * cos(pos_theta)
        pos_y += odemetry * sin(pos_theta)
        self.pos = (pos_x, pos_y, pos_theta)

        # Extract landmarks.
        spike_landmarks = self.extract_spike(observations)
        ransac_landmarks = self.extract_ransac(observations)

        # Associate landmarks.
        associated_landmarks = self.associate_landmarks(LandmarkType.SPIKE, spike_landmarks)
        associated_landmarks += self.associate_landmarks(LandmarkType.RANSAC, ransac_landmarks)

        # TODO(Daniel): Update the EKF and use that to further refine belief.

        # Update point map.
        for i, reading in enumerate(raw_data):
            observed_x = pos_x + (reading * cos(pos_theta - radians(i)))
            observed_y = pos_y + (reading * sin(pos_theta - radians(i)))
            self.map.append((observed_x, observed_y))
            # TODO(Daniel): This will consume memory proportional to runtime, consider doing something nicer.

    def associate_landmarks(self, landmark_type, landmarks):
        '''Find existing landmarks which match the observed landmarks, or add new ones if necessary.
           Return a list of (landmark, new_landmark_observation) pairs.'''
        associated_landmarks = []
        for landmark_pos in landmarks:
            lx, ly = landmark_pos
            associated_landmark = self.landmarks.find_nearest(landmark_type, lx, ly)
            landmark_dist = (sqrt((lx - associated_landmark.x) ** 2 + (ly - associated_landmark.y) ** 2)
                             if associated_landmark is not None
                             else inf)
            if landmark_dist > self.landmark_association_threshold:
                # Existing landmark was too far away, so create a new one.
                associated_landmark = Landmark(landmark_type, lx, ly)
                self.landmarks.insert_landmark(associated_landmark)
            else:
                # The existing landmark was close enough, so count it as seen again.
                associated_landmark.increment_seen()

            associated_landmarks.append((associated_landmark, landmark_pos))

        return associated_landmarks

    def extract_spike(self, raw_data):
        '''Observe environment and extract landmarks using the spike technique.'''

        landmarks = []
        x, y, theta = self.pos

        for i, B in enumerate(raw_data):
            A = raw_data[(i - 1) % len(raw_data)]
            C = raw_data[(i + 1) % len(raw_data)]

            if A >= 0 and B >= 0 and C >= 0 and ((A - B) + (C - B)) >= self.spike_threshold:
                mark_x = x + (B * cos(theta - radians(i)))
                mark_y = y + (B * sin(theta - radians(i)))

                landmarks.append((mark_x, mark_y))

        return landmarks

    def extract_ransac(self, raw_data, seed_override=None):
        '''Observe environment and extract landmarks using the RANSAC technique.'''

        # allow setting the random seed for predictable testing
        if seed_override is not None:
            seed(seed_override)

        landmarks = []
        x, y, theta = self.pos

        # process the data into points and angles
        data = [(x + (distance * cos(theta - radians(angle))),
                 y + (distance * sin(theta - radians(angle))),
                 angle) for angle, distance in
                enumerate(raw_data)]

        associated = set()
        for i in range(self.ransac_max_tries):
            if len(data) - len(associated) < self.ransac_consensus:
                # if there are not enough points to reach consensus, quit
                break

            # select a base point for the fit
            point = choice([p for p in data if not p[2] in associated])

            # define the start and end for the considered range
            start = (point[2] - self.ransac_range) % len(data)
            end = (point[2] + self.ransac_range + 1) % len(data)

            # generate a list of unclaimed points within the range
            if start < end:
                possible_points = [p for p in data[start:end]
                                   if not p[2] in associated and p[2] != point[2]]
            else:
                possible_points = [p for p in data[start:] + data[:end]
                                   if not p[2] in associated and p[2] != point[2]]

            if len(possible_points) < (self.ransac_samples - 1):
                # if there are not enough points for the fit, restart
                continue

            # select configured number of points, always including the base point
            points = [point] + sample(possible_points, self.ransac_samples - 1)

            # generate best fit
            line = linear_regression(points)
            # TODO(Daniel): transition to using 'total least squares'

            # restart if line doesn't meet consensus
            supporters = []
            for point in data:
                if not point[2] in associated and min_distance(line, point) < self.ransac_error:
                    supporters.append(point)
            if len(supporters) < self.ransac_consensus:
                continue

            # associate all points close enough
            for point in supporters:
                associated.add(point[2])

            # regenerate best fit using all points close enough
            line = linear_regression(supporters)

            # calculate landmark representation, add to landmarks list
            landmarks.append(closest_point(line, (0,0)))

        return landmarks
