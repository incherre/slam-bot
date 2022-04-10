'''An implementation of SLAM techniques.'''

from abc import ABC, abstractmethod
from enum import Enum, auto, unique

from ekf import EKF
from landmark_extraction import extract_spike, extract_ransac
from collision_map import CollisionMap

# Option dictionary constants, just for reference.
SPIKE_THRESHOLD = 'spike_threshold'

RANSAC_MAX_TRIES = 'ransac_max_tries'
RANSAC_SAMPLES = 'ransac_samples'
RANSAC_RANGE = 'ransac_range'
RANSAC_ERROR = 'ransac_error'
RANSAC_CONSENSUS = 'ransac_consensus'

EKF_INITIAL_UNCERTAINTY = 'ekf_initial_uncertainty'
EKF_ODOMETRY_NOISE = 'ekf_odometry_noise'
EKF_RANGE_NOISE = 'ekf_range_noise'
EKF_BEARING_NOISE = 'ekf_bearing_noise'
EKF_INNOVATION_LAMBDA = 'ekf_innovation_lambda'
EKF_LANDMARK_THRESHOLD = 'ekf_landmark_threshold'

COLLISION_MAP_SCALE = 'collision_map_scale'
COLLISION_MAP_MAX_DISTANCE = 'collision_map_max_dist'


class SensingAndControl(ABC):
    '''The abstract class for obtaining readings and controlling the robot.'''

    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def get_distance_reading(self):
        '''Should return a list of distance readings evenly spaced,
           in order, without any gaps. In the format of
           [(relative_theta, distance), ...].'''
        pass

    @abstractmethod
    def move(self, theta, distance):
        '''Turn theta then go distance and return odemetry.'''
        pass

@unique
class LandmarkType(Enum):
    SPIKE = auto()
    RANSAC = auto()

class Slam:
    '''Manages a Slam instance.'''

    def __init__(self, control, option_dictionary={}):
        self.control = control
        self.ekf = EKF(**option_dictionary)
        self.map = CollisionMap(**option_dictionary)
        self.option_dictionary = option_dictionary

    def get_estimated_position(self):
        '''Returns the current estimate of the bot's position.'''
        [x], [y], [theta] = self.ekf.pos()
        return (x, y, theta)

    def move_observe_and_update(self, theta, distance):
        '''Turn theta then go distance, make a new observation, and update estimated location.'''
        # Issue commands to robot and recieve instrument readings.
        odemetry = self.control.move(theta, distance)
        observations = self.control.get_distance_reading()

        # Extract landmarks.
        spike_landmarks = extract_spike(self.get_estimated_position(), observations, **self.option_dictionary)
        ransac_landmarks = extract_ransac(self.get_estimated_position(), observations, **self.option_dictionary)

        # Label and combine landmarks.
        landmarks = [(x, y, LandmarkType.SPIKE) for x, y in spike_landmarks]
        landmarks += [(x, y, LandmarkType.RANSAC) for x, y in ransac_landmarks]

        # Update EKF.
        self.ekf.update(theta, odemetry, landmarks)

        # Update collision map.
        x, y, theta = self.get_estimated_position()
        self.map.record_observations(x, y, theta, observations)

    def get_collision_map(self):
        '''Returns the CollisionMap object containing environmental data.'''
        return self.map
