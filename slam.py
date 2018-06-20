'''An implementation of SLAM techniques.'''
import sim_framework
from abc import ABC, abstractmethod
from math import radians, sin, cos

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

    def __init__(self, control):
        self.control = control
        self.landmarks = dict()
        self.pos = (0, 0, 0) # (x, y, theta)
        self.map = []

    def extract_spike(self):
        '''Observe environment and extract landmarks using the spike technique.'''

        data = self.control.get_distance_reading()
        landmarks = []
        for i, B in enumerate(data):
            A = data[(i - 1) % len(data)]
            C = data[(i + 1) % len(data)]

            if A >= 0 and B >= 0 and C >= 0 and abs((A - B) + (C - B)) >= self.spike_threshold:
                x = self.pos[0] + (B * cos(self.pos[2] - radians(i)))
                y = self.pos[1] + (B * sin(self.pos[2] - radians(i)))

                landmarks.append((x, y))

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
