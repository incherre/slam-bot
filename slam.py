'''An implementation of SLAM techniques.'''
import sim_framework
from abc import ABC, abstractmethod
from math import radians

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
