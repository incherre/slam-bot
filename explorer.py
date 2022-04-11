'''A controller built on top of the SLAM implementation that will target un-explored areas.'''

from enum import Enum, auto, unique
from collections import deque
from math import atan2, sqrt

from slam import Slam
from landmark_extraction import normalized_angle_difference

@unique
class SpaceStatus(Enum):
    UNKNOWN = auto()
    PASSABLE = auto()
    BLOCKED = auto()

class Explorer:
    '''A robot controller that uses SLAM to explore its environment.'''

    def __init__(self, slam, step_threshold=1, miss_threshold=5, hit_threshold=2):
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

    def step(self, debug = False):
        '''Runs one step of exploration.'''
        x, y, theta = self.slam.get_estimated_position()
        collision_map = self.slam.get_collision_map()

        target = None
        current_location = collision_map.get_key(x, y)

        open_queue = deque()

        # Key is a location already seen, value is the location to get to that location from.
        closed_trail = {}

        open_queue.append(current_location)
        closed_trail[current_location] = None

        while len(open_queue) > 0:
            current_x, current_y = open_queue.popleft()
            current_status = self.get_space_status(collision_map.get_location(current_x, current_y))

            if current_status == SpaceStatus.BLOCKED:
                continue

            if current_status == SpaceStatus.UNKNOWN:
                target = (current_x, current_y)
                break

            for next_key in collision_map.get_neighbor_keys(current_x, current_y):
                if current_x != next_key[0] and current_y != next_key[1]:
                    # Don't try to squeeze through corners!
                    continue

                if not next_key in closed_trail:
                    open_queue.append(next_key)
                    closed_trail[next_key] = (current_x, current_y)

        if target is None:
            # Everything accessible is already explored.
            if debug:
                print("Nowhere unexplored.")
            return

        if target == current_location:
            # Haven't observed where we are now, probably the first iteration.
            self.slam.move_observe_and_update(0, 0)
            if debug:
                print("Present location unexplored.")
            return

        if debug:
            print("Going to explore:", target)

        # Find the first step on the path to the target.
        while closed_trail[target] is not None and closed_trail[closed_trail[target]] is not None:
            target = closed_trail[target]

        if debug:
            print("by way of:", target)

        # Compute the controls to take that step.
        dx = target[0] - x
        dy = target[1] - y

        target_theta = atan2(dy, dx)
        target_distance = sqrt(dx ** 2 + dy ** 2)

        theta_controls = normalized_angle_difference(theta, target_theta)

        self.slam.move_observe_and_update(theta_controls, target_distance)

    def get_space_status(self, space):
        '''Returns the status of a given space.'''
        if space.stepped_count >= self.step_threshold:
            return SpaceStatus.PASSABLE

        if space.hit_count >= self.hit_threshold:
            return SpaceStatus.BLOCKED

        if space.missed_count >= self.miss_threshold:
            return SpaceStatus.PASSABLE

        return SpaceStatus.UNKNOWN
