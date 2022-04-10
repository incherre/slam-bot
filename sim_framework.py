'''Robot simulation for testing SLAM techniques.'''
from abc import ABC, abstractmethod
from math import sqrt, sin, cos, floor, ceil, radians, inf
from slam import SensingAndControl

class World:
    '''The World class manages all the simulation resources.'''

    def __init__(self, resolution=0.01, max_dist=1000, collision_delta_theta=1):
        self.obstacles = []
        self.entities = []
        self.resolution = resolution
        self.max_dist = max_dist
        self.collision_delta_theta = collision_delta_theta

    def add_obs(self, obstacle):
        '''Adds an obstacle to the list.'''
        self.obstacles.append(obstacle)

    def add_ent(self, entity):
        '''Adds an entitiy to the list.'''
        self.entities.append(entity)

    def ray_cast(self, x, y, theta):
        '''Returns the distance until a collision from (x, y) in the theta direction.'''
        x_inc = self.resolution * cos(theta)
        y_inc = self.resolution * sin(theta)
        
        current_x = x
        current_y = y
        dist = 0

        while dist < self.max_dist:
            for obstacle in self.obstacles:
                if obstacle.is_inside(current_x, current_y):
                    return dist

            dist += self.resolution
            current_x += x_inc
            current_y += y_inc

        return inf

    def move_ent(self, entity, distance, theta):
        '''Attempts to move entity distance in the theta direction, returns whether there was a collision.'''
        clearance = 0
        x, y, ent_theta = entity.get_pos()
        x_inc = self.resolution * cos(theta)
        y_inc = self.resolution * sin(theta)
        current_x = x
        current_y = y

        while entity.is_inside(current_x, current_y):
            clearance += self.resolution
            current_x += x_inc
            current_y += y_inc

        current_x += x_inc / 2
        current_y += y_inc / 2
        if entity.is_inside(current_x, current_y):
            clearance += self.resolution
        else:
            clearance += self.resolution / 2

        collision_distance = min(
            self.ray_cast(x, y, theta - self.collision_delta_theta),
            self.ray_cast(x, y, theta),
            self.ray_cast(x, y, theta + self.collision_delta_theta))
        max_distance = max(collision_distance - clearance, 0)
        real_distance = min(max_distance, distance)

        entity.set_pos(x + (real_distance * cos(theta)), y + (real_distance * sin(theta)), ent_theta)

        entity.record_move(real_distance, theta)
        return real_distance < distance

    def display(self, resolution):
        '''Displays the environment, by default just as a character array.'''
        x_max = max(map(lambda obstacle: obstacle.center_point()[0]
                        if obstacle.center_point()[0] is not None else -inf, self.obstacles))
        x_min = min(map(lambda obstacle: obstacle.center_point()[0]
                        if obstacle.center_point()[0] is not None else inf, self.obstacles))
        y_max = max(map(lambda obstacle: obstacle.center_point()[1]
                        if obstacle.center_point()[1] is not None else -inf, self.obstacles))
        y_min = min(map(lambda obstacle: obstacle.center_point()[1]
                        if obstacle.center_point()[1] is not None else inf, self.obstacles))

        y = y_max
        while y >= y_min:
            x = x_min
            while x <= x_max:
                for i in range(len(self.entities)):
                    if self.entities[i].is_inside(x, y):
                        print(i % 10, end='')
                        break
                else:
                    for obs in self.obstacles:
                        if obs.is_inside(x, y):
                            print('#', end='')
                            break
                    else:
                        print(' ', end='')
                x += resolution
            print()
            y -= resolution

class Obstacle(ABC):
    '''A visual and physical obstruction in a World.'''

    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def is_inside(self, x, y):
        '''Returns whether the point (x, y) is inside self.'''
        pass

    @abstractmethod
    def center_point(self):
        '''Returns the center point (x, y) of the obstacle.
           Either x or y may be None to indicate unbounded size in that dimension.'''
        pass

class Wall(Obstacle):
    '''An infinite horizontal or vertical border.'''

    def __init__(self, position, option):
        super().__init__()

        self.threshold = position

        if option.lower() in ['+x', '-x', '+y', '-y']:
            self.option = option.lower()
        else:
            raise ValueError('Invalid option parameter. Accepted values are: \'+x\', \'-x\', \'+y\', \'-y\'')

    def is_inside(self, x, y):
        '''Returns whether the point is inside the wall.'''
        if self.option[1] == 'x':
            pos = x
        else:
            pos = y

        if self.option[0] == '-':
            return pos <= self.threshold
        else:
            return pos >= self.threshold

    def center_point(self):
        '''Returns a point-like object on the threshold of the wall with only the relevant axis defined.'''
        if self.option[1] == 'x':
            return (self.threshold, None)
        else:
            return (None, self.threshold)

class Box(Obstacle):
    '''A square to get in the way of things.'''

    def __init__(self, x_min, x_max, y_min, y_max):
        super().__init__()

        if x_min >= x_max:
            raise ValueError('Improperly ordered x boundaries')
        self.x_min = x_min
        self.x_max = x_max

        if y_min >= y_max:
            raise ValueError('Improperly ordered y boundaries')
        self.y_min = y_min
        self.y_max = y_max

    def is_inside(self, x, y):
        '''Returns whether the point is inside the box.'''
        ans = x >= self.x_min and x <= self.x_max
        ans = ans and y >= self.y_min and y <= self.y_max
        return ans

    def center_point(self):
        '''Returns the center point of the box.'''
        return ((self.x_min + self.x_max) / 2, (self.y_min + self.y_max) / 2)

class Entity(ABC):
    '''A mobile agent within a World.'''

    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

    def get_pos(self):
        '''Returns the position of the entity as (x, y, theta).'''
        return (self.x, self.y, self.theta)

    def set_pos(self, x, y, theta):
        '''Sets the position of the entity as (x, y, theta).'''
        self.x = x
        self.y = y
        self.theta = theta % radians(360)

    @abstractmethod
    def record_move(self, distance, theta):
        '''Records an attempted move, used by World.move_ent.'''
        pass

    @abstractmethod
    def is_inside(self, x, y):
        '''Returns whether the point (x, y) is inside self.'''
        pass

class CircleBot(Entity):
    '''A simple circular entity that can keep track of distance traveled.'''

    def __init__(self, radius, x, y, theta):
        super().__init__()

        self.odemetry = 0
        self.radius = radius
        self.set_pos(x, y, theta)

    def record_move(self, distance, theta):
        '''Records the distance attempted. Assume wheel slippage if it runs into an obstacle.'''
        self.odemetry += distance

    def get_odemetry(self):
        '''Returns the current distance sum.'''
        return self.odemetry

    def reset_odemetry(self):
        '''Resets the distance counter to zero.'''
        self.odemetry = 0

    def is_inside(self, x, y):
        '''Checks if (x, y) is within self.radius of self.'''
        distance = sqrt((self.x - x) ** 2 + (self.y - y) ** 2)
        return distance <= self.radius

class SimBotControl(SensingAndControl):
    '''A robot in a simulation.'''

    def __init__(self, world, bot, arc_degrees = 1):
        super().__init__()

        self.world = world
        self.bot = bot

        assert(arc_degrees > 0 and arc_degrees < 360 and 360 % arc_degrees == 0)
        self.arc_degrees = arc_degrees

    def get_distance_reading(self):
        '''Returns the distance reading.'''
        #TODO(Daniel): add sensor noise

        reading = []
        x, y, theta = self.bot.get_pos()

        for i in range(-180, 180, self.arc_degrees):
            reading.append((radians(i), self.world.ray_cast(x, y, theta + radians(i))))

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

if __name__ == '__main__':
    W = World()
    W.add_obs(Wall(-5, '-x'))
    W.add_obs(Wall(5, '+x'))
    W.add_obs(Wall(-5, '-y'))
    W.add_obs(Wall(5, '+y'))
    W.add_obs(Box(-5.5, -3, 3, 5.5))

    bot = CircleBot(1, 0, 0, 0)
    W.add_ent(bot)
    
    W.display(0.5)
    W.move_ent(bot, 2, bot.theta)
    W.display(0.5)
    W.move_ent(bot, 20, bot.theta)
    W.display(0.5)
