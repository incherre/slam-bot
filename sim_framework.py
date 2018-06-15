'''Robot simulation for testing SLAM techniques.'''
from abc import ABC, abstractmethod
from math import sqrt, sin, cos, floor, ceil

class World:
    '''The World class manages all the simulation resources.'''

    def __init__(self):
        self.obstacles = []
        self.entities = []
        self.resolution = 0.01
        self.max_dist = 1000

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

        return -1

    def move_ent(self, entity, distance, theta):
        '''Attempts to move entity distance in the theta direction, keeping in mind collisions.'''
        entity.record_move(distance, theta)

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

        max_distance = max(self.ray_cast(x, y, theta) - clearance, 0)
        real_distance = min(max_distance, distance)

        entity.set_pos(x + (real_distance * cos(theta)), y + (real_distance * sin(theta)), ent_theta)

    def display(self, x_max, x_min, y_max, y_min, resolution):
        '''Displays the environment, by default just as a character array.'''
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
        self.theta = theta

    @abstractmethod
    def record_move(self, distance, theta):
        '''Records an attempted move, used by World.move_ent.'''
        pass

    @abstractmethod
    def is_inside(self, x, y):
        '''Returns whether the point (x, y) is inside self.'''
        pass

class CircleBot(Entity):
    '''A simple circular entity that can keep track of ~distance traveled.'''

    def __init__(self, radius, x, y, theta):
        super().__init__()

        self.odemetry = 0
        self.radius = radius
        self.set_pos(x, y, theta)

    def record_move(self, distance, theta):
        '''Records the distance attempted. Assume wheel slippage if it runs into an obstacle.'''
        self.odemetry += distance #TODO(Daniel): Add noise of some kind

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

if __name__ == '__main__':
    W = World()
    W.add_obs(Wall(-5, '-x'))
    W.add_obs(Wall(5, '+x'))
    W.add_obs(Wall(-5, '-y'))
    W.add_obs(Wall(5, '+y'))
    W.add_obs(Box(-5.5, -3, 3, 5.5))

    bot = CircleBot(1, 0, 0, 0)
    W.add_ent(bot)
    
    W.display(5.5, -5.5, 5.5, -5.5, 0.5)
    W.move_ent(bot, 2, bot.theta)
    W.display(5.5, -5.5, 5.5, -5.5, 0.5)
    W.move_ent(bot, 20, bot.theta)
    W.display(5.5, -5.5, 5.5, -5.5, 0.5)
