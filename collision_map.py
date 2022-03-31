'''A map to record possible obstacles.'''

from math import sin, cos, inf, sqrt

class MapLocation:
    '''A location of the map that stores various counts.'''
    def __init__(self):
        # The number of times an agent has stepped in this location, strong evidence it is clear.
        self.stepped_count = 0

        # The number of times a scan beam has passed through this location, weak evidence it is clear.
        self.missed_count = 0

        # The number of times a scan beam has terminated in this location, evidence of an obstacle.
        self.hit_count = 0

    def __eq__(self, other):
        return isinstance(other, MapLocation) and \
            self.stepped_count == other.stepped_count and \
            self.missed_count == other.missed_count and \
            self.hit_count == other.hit_count

def get_discrete_coord(scale, coord):
    shifted = coord + (scale / 2)
    return int(shifted - shifted % scale)

class CollisionMap:
    '''A map which records possible obstacles given a sensor reading.'''
    def __init__(self, scale=5, max_dist=100):
        assert(isinstance(scale, int))
        assert(scale > 0)
        self.scale = scale

        
        assert(isinstance(max_dist, int))
        assert(max_dist > 0)
        self.max_dist = max_dist

        self.map = {}

    def get_location(self, x, y, create = False):
        '''Retrieves the obstacle information for the given location.'''
        key = self.__get_key(x, y)
        if not key in self.map and not create:
            return MapLocation()

        if not key in self.map and create:
            self.map[key] = MapLocation()

        return self.map[key]

    def get_neighbor_keys(self, x, y):
        '''Get the keys for all 8 adjacent neighbor cells.'''
        key_x, key_y = self.__get_key(x, y)
        neighbors = []
        
        for dx in [-self.scale, 0, self.scale]:
            for dy in [-self.scale, 0, self.scale]:
                if dx == 0 and dy == 0:
                    continue

                neighbors.append((key_x + dx, key_y + dy))

        return neighbors

    def record_observations(self, x, y, theta, observations):
        '''Record the appropriate counts for each of the given (delta_theta, distance) pairs based out
           of the provided current location.'''

        current_location_key = self.__get_key(x, y)
        self.__get_or_insert_location(current_location_key).stepped_count += 1

        for delta_theta, distance in observations:
            self.__add_line(x, y, theta + delta_theta, distance, current_location_key)

    def __add_line(self, start_x, start_y, theta, distance, current_location_key):
        '''Record all spots along the given line as passed through, and records the final spot as hit.'''
        start_point_key = self.__get_key(start_x, start_y)
        end_x = start_x + min(distance, self.max_dist) * cos(theta)
        end_y = start_y + min(distance, self.max_dist) * sin(theta)
        end_point_key = self.__get_key(end_x, end_y)
        
        if distance <= self.max_dist:
            self.__get_or_insert_location(end_point_key).hit_count += 1

        x, y = start_point_key
        current_distance = sqrt((end_x - x) ** 2 + (end_y - y) ** 2)

        a = -sin(theta)
        b = cos(theta)
        c = (start_x * sin(theta)) - (start_y * cos(theta))

        while current_distance <= self.max_dist:
            if self.__get_key(x, y) != start_point_key:
                # Don't record the starting point as passed through,
                # since it will be recorded as stepped in.
                self.get_location(x, y, create = True).missed_count += 1

            if (x, y) in self.get_neighbor_keys(end_point_key[0], end_point_key[1]):
                break

            next_x = None
            next_y = None
            next_error = inf

            for option_x, option_y in self.get_neighbor_keys(x, y):
                option_dist = sqrt((end_x - option_x) ** 2 + (end_y - option_y) ** 2)

                if option_dist >= current_distance:
                    # Don't move away from the target!
                    continue

                option_error = abs((a * option_x) + (b * option_y) + c) / sqrt(a ** 2 + b ** 2)
                if option_error < next_error:
                    next_x = option_x
                    next_y = option_y
                    next_error = option_error

            if next_x is None or next_y is None:
                break

            x = next_x
            y = next_y
            current_distance = sqrt((end_x - x) ** 2 + (end_y - y) ** 2)

    def __get_key(self, x, y):
        '''Returns the key into the internal map corresponding to the provided point.'''
        shifted_x = x + (self.scale / 2)
        shifted_y = y + (self.scale / 2)
        return (int(shifted_x - shifted_x % self.scale), int(shifted_y - shifted_y % self.scale))

    def __get_or_insert_location(self, key):
        '''Retrieves the obstacle information for the given location, but will insert
           a new location into the map if it is missing.'''
        if not key in self.map:
            self.map[key] = MapLocation()

        return self.map[key]