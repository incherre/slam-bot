'''A controller built on top of the SLAM implementation that will target un-explored areas.'''

from enum import Enum, auto, unique
from heapq import heappush, heappop
from math import atan2, sqrt, sin, cos, degrees
import numpy as np

from slam import Slam
from landmark_extraction import normalized_angle_difference

@unique
class SpaceStatus(Enum):
    UNKNOWN = auto()
    PASSABLE = auto()
    BLOCKED = auto()

class Explorer:
    '''A robot controller that uses SLAM to explore its environment.'''

    def __init__(self, slam, size, step_threshold=1, miss_threshold=5, hit_threshold=2):
        assert(hasattr(slam, "get_estimated_position"))
        assert(hasattr(slam, "get_collision_map"))
        assert(hasattr(slam, "move_observe_and_update"))
        self.slam = slam

        assert(isinstance(step_threshold, int))
        self.step_threshold = step_threshold

        assert(isinstance(miss_threshold, int))
        self.miss_threshold = miss_threshold

        assert(isinstance(hit_threshold, int))
        self.hit_threshold = hit_threshold

        self.size = size
        self.current_path = []
        self.fully_explored = False

    def step(self, debug = False):
        '''Runs one step of exploration.'''
        if self.fully_explored:
            # Don't do a bunch of expensive pathfinding if we've explored everything already.
            return

        x, y, theta = self.slam.get_estimated_position()
        collision_map = self.slam.get_collision_map()

        recompute_path = len(self.current_path) == 0
        recompute_path = recompute_path or self.get_space_status(
            collision_map.get_location(self.current_path[0][0],
                                       self.current_path[0][1])) != SpaceStatus.UNKNOWN
        recompute_path = recompute_path or self.get_travel_status(
            x, y,
            self.current_path[-1][0],
            self.current_path[-1][1]) == SpaceStatus.BLOCKED

        if recompute_path:
            self.current_path = self.pathfind()

        if len(self.current_path) == 0:
            need_to_observe = self.get_space_status(collision_map.get_location(x, y)) == SpaceStatus.UNKNOWN
            if debug and need_to_observe:
                print("Present location unexplored.")
            elif debug:
                print("Nowhere unexplored.")

            if need_to_observe:
                # Haven't observed where we are now, probably the first iteration.
                self.slam.move_observe_and_update(0, 0)
            else:
                # Everything accessible is already explored.
                self.fully_explored = True
            return

        if debug:
            print("Going to explore:", self.current_path[0])

        if debug:
            print("by way of:", self.current_path[-1])

        # Compute the controls to take that step.
        next_step = self.current_path.pop()
        dx = next_step[0] - x
        dy = next_step[1] - y

        target_theta = atan2(dy, dx)
        target_distance = sqrt(dx ** 2 + dy ** 2)

        theta_controls = normalized_angle_difference(theta, target_theta)

        self.slam.move_observe_and_update(theta_controls, target_distance)

    def pathfind(self):
        '''Finds a path to the closest space with "unknown" status.'''

        x, y, theta = self.slam.get_estimated_position()
        collision_map = self.slam.get_collision_map()

        target = None
        current_location = collision_map.get_key(x, y)

        # Contains tuples of (path_distance, (x, y)).
        open_heap = []

        # Key is a location already seen, value is the location to get to that location from.
        closed_trail = {}

        heappush(open_heap, (0, current_location))
        closed_trail[current_location] = None

        while len(open_heap) > 0:
            path_distance, [current_x, current_y] = heappop(open_heap)
            current_status = self.get_space_status(collision_map.get_location(current_x, current_y))

            if current_status == SpaceStatus.BLOCKED:
                continue

            if current_status == SpaceStatus.UNKNOWN:
                target = (current_x, current_y)
                break

            for next_key in collision_map.get_neighbor_keys(current_x, current_y):
                if self.get_travel_status(current_x, current_y, next_key[0], next_key[1]) == SpaceStatus.BLOCKED:
                    continue

                new_distance = path_distance + sqrt((current_x - next_key[0])**2 + (current_y - next_key[1])**2)

                if not next_key in closed_trail:
                    heappush(open_heap, (new_distance, next_key))
                    closed_trail[next_key] = (current_x, current_y)

        if target is None:
            return []

        path = []
        current_step = target

        while closed_trail[current_step] is not None:
            path.append(current_step)
            current_step = closed_trail[current_step]

        return path

    def get_travel_status(self, x1, y1, x2, y2):
        '''Returns the most prohibitive status within a given movement.'''
        travel_angle = atan2(y2 - y1, x2 - x1)
        rotate_mat = np.array([[cos(travel_angle), -sin(travel_angle)],
                               [sin(travel_angle), cos(travel_angle)]])

        start_point_1 = (rotate_mat @ (np.array([[-1], [1]]) * (self.size / 2))) + np.array([[x1], [y1]])
        start_point_2 = (rotate_mat @ (np.array([[-1], [-1]]) * (self.size / 2))) + np.array([[x1], [y1]])
        end_point_1 = (rotate_mat @ (np.array([[1], [-1]]) * (self.size / 2))) + np.array([[x2], [y2]])
        end_point_2 = (rotate_mat @ (np.array([[1], [1]]) * (self.size / 2))) + np.array([[x2], [y2]])

        start_point_1 = tuple(np.transpose(start_point_1)[0])
        start_point_2 = tuple(np.transpose(start_point_2)[0])
        end_point_1 = tuple(np.transpose(end_point_1)[0])
        end_point_2 = tuple(np.transpose(end_point_2)[0])

        collision_map = self.slam.get_collision_map()
        spaces, area = collision_map.get_locations_within_rectangle(
            start_point_1, start_point_2, end_point_1, end_point_2)

        status = SpaceStatus.PASSABLE
        if len(spaces) < area:
            status = SpaceStatus.UNKNOWN

        for [x, y] in spaces:
            this_status = self.get_space_status(collision_map.get_location(x, y))

            if this_status == SpaceStatus.UNKNOWN and status == SpaceStatus.PASSABLE:
                status = this_status
            elif this_status == SpaceStatus.BLOCKED:
                status = this_status
        
        return status

    def get_space_status(self, space):
        '''Returns the status of a given space.'''
        if space.stepped_count >= self.step_threshold:
            return SpaceStatus.PASSABLE

        if space.hit_count >= self.hit_threshold:
            return SpaceStatus.BLOCKED

        if space.missed_count >= self.miss_threshold:
            return SpaceStatus.PASSABLE

        return SpaceStatus.UNKNOWN
