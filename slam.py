'''An implementation of SLAM techniques.'''
import sim_framework
from abc import ABC, abstractmethod
from math import radians, sin, cos
from random import choice, sample

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

class SimBot(SensingAndControl):
    '''A robot in a simulation.'''

    def __init__(self, world, bot):
        super().__init__()

        self.world = world
        self.bot = bot

    def get_distance_reading(self):
        '''Returns the distance reading.'''
        #TODO(Daniel): add sensor noise

        reading = []
        x, y, theta = self.bot.get_pos()

        for i in range(360):
            reading.append(self.world.ray_cast(x, y, theta - radians(i)))

        return reading

    def move(self, theta, distance):
        '''Turn theta and go distance.'''
        #TODO(Daniel): add sensor / control noise

        x, y, old_theta = self.bot.get_pos()
        new_theta = old_theta + theta
        self.bot.set_pos(x, y, new_theta)
        self.bot.reset_odemetry()

        self.world.move_ent(self.bot, distance, new_theta)

        return self.bot.get_odemetry()

class Slam:
    '''Manages a Slam instance.'''

    spike_threshold = 0.5

    ransac_max_tries = 500
    ransac_samples = 5
    ransac_range = 10
    ransac_error = 0.5
    ransac_consensus = 15

    def __init__(self, control):
        self.control = control
        self.landmarks = dict()
        self.pos = (0, 0, 0) # (x, y, theta)
        self.map = []

    def extract_spike(self):
        '''Observe environment and extract landmarks using the spike technique.'''

        raw_data = self.control.get_distance_reading()
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

    def extract_ransac(self):
        '''Observe environment and extract landmarks using the RANSAC technique.'''

        landmarks = []
        x, y, theta = self.pos

        # process the data into points and angles
        data = [(x + (distance * cos(theta - radians(angle))),
                 y + (distance * sin(theta - radians(angle))),
                 angle) for angle, distance in
                enumerate(self.control.get_distance_reading())]

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
                # if there are not enough points for the fit, quit
                break

            # select configured number of points, always including the base point
            points = [point] + sample(possible_points, self.ransac_samples - 1)

            # TODO(Daniel): generate best fit

            # TODO(Daniel): break if line doesn't meet consensus

            # TODO(Daniel): associate all points close enough

            # TODO(Daniel): regenerate best fit using all points close enough

            # TODO(Daniel): calculate landmark representation, add to landmarks list

        return landmarks

if __name__ == "__main__":
    W = sim_framework.World()
    W.add_obs(sim_framework.Wall(-5, '-x'))
    W.add_obs(sim_framework.Wall(5, '+x'))
    W.add_obs(sim_framework.Wall(-5, '-y'))
    W.add_obs(sim_framework.Wall(5, '+y'))
    W.add_obs(sim_framework.Box(-5.5, -3, 3, 5.5))
    W.add_obs(sim_framework.Box(3, 3.2, -3.2, -3))

    bot = sim_framework.CircleBot(1, 0, 0, 0)
    W.add_ent(bot)

    slam = Slam(SimBot(W, bot))

    print(slam.extract_spike())
    W.display(5.5, -5.5, 5.5, -5.5, 0.5)
