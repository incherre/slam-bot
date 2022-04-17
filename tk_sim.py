'''Robot sim with a nicer display.'''

from math import radians
from datetime import datetime
import tkinter
import os

from sim_framework import *
import slam
from explorer import Explorer

BACKGROUND_COLOR = 'grey60'
ENTITY_COLOR = 'RoyalBlue1'
OBSTACLE_COLOR = 'black'
ESTIMATED_OBSTACLE_COLOR = 'red'
ESTIMATED_OBSTACLE_TAG = 'estimated_obstacle'
ESTIMATED_POS_COLOR = 'green'
ESTIMATED_POS_TAG = 'estimated_pos'
ENTITY_TAG = 'entity'

class TKWorld(World):
    '''A world that will display via tkinter instead of ascii.'''
    def __init__(self, root, x_min, x_max, y_min, y_max, resolution=2, max_dist=1414, collision_delta_theta=1):
        super().__init__(resolution=resolution, max_dist=max_dist, collision_delta_theta=collision_delta_theta)
        if x_min >= x_max:
            raise ValueError('Improperly ordered x boundaries')
        self.x_min = x_min
        self.x_max = x_max

        if y_min >= y_max:
            raise ValueError('Improperly ordered y boundaries')
        self.y_min = y_min
        self.y_max = y_max

        self.root = root
        self.room_canvas = tkinter.Canvas(self.root, bg=BACKGROUND_COLOR,
                                          height=self.y_max - self.x_min,
                                          width=self.x_max - self.x_min)

        self.add_obs(Wall(self.x_min, '-x'))
        self.add_obs(Wall(self.x_max, '+x'))
        self.add_obs(Wall(self.y_min, '-y'))
        self.add_obs(Wall(self.y_max, '+y'))

    def add_obs(self, obstacle):
        '''Adds the obstacle to tracking and also the TK canvas.'''
        super().add_obs(obstacle)

        if isinstance(obstacle, Wall):
            # In the TK world, walls are only used for the outside of the box.
            pass
        elif isinstance(obstacle, Box):
            box_x1, box_y1 = self.get_canvas_coords(obstacle.x_min, obstacle.y_max)
            box_x2, box_y2 = self.get_canvas_coords(obstacle.x_max, obstacle.y_min)
            self.room_canvas.create_rectangle(box_x1, box_y1,
                                              box_x2, box_y2,
                                              fill=OBSTACLE_COLOR, outline=OBSTACLE_COLOR)
        else:
            print('Error: Unknown obstacle type added to sim:', type(obstacle).__name__)

    def get_canvas_coords(self, x, y):
        '''Converts simulation coordinates to canvas coordinates.'''
        disp_x = x - self.x_min
        disp_y = (self.y_max - self.y_min) - (y - self.y_min) - 1

        return (disp_x, disp_y)

    def display(self, collision_map={}, estimated_pos=None, mark_radius=10):
        '''Displays the environment.'''
        try:
            self.room_canvas.delete(ENTITY_TAG)
            self.room_canvas.delete(ESTIMATED_OBSTACLE_TAG)
            self.room_canvas.delete(ESTIMATED_POS_TAG)
        except _tkinter.TclError:
            return

        for ent in self.entities:
            if isinstance(ent, CircleBot):
                center_x, center_y = self.get_canvas_coords(ent.x, ent.y)
                ent_x1 = center_x - ent.radius
                ent_y1 = center_y - ent.radius
                ent_x2 = center_x + ent.radius
                ent_y2 = center_y + ent.radius
                
                self.room_canvas.create_oval(ent_x1, ent_y1,
                                             ent_x2, ent_y2,
                                             fill=ENTITY_COLOR, outline=ENTITY_COLOR,
                                             tags=(ENTITY_TAG,))
            else:
                print('Error: Unknown entity type found in sim:', type(ent).__name__)

        for [obs_x, obs_y], properties in collision_map.items():
            x, y = self.get_canvas_coords(obs_x, obs_y)
            if properties.hit_count > 1:
                x1 = x - mark_radius
                y1 = y - mark_radius
                x2 = x + mark_radius
                y2 = y + mark_radius
                self.room_canvas.create_oval(x1, y1,
                                             x2, y2,
                                             fill=ESTIMATED_OBSTACLE_COLOR,
                                             outline=ESTIMATED_OBSTACLE_COLOR,
                                             tags=(ESTIMATED_OBSTACLE_TAG,))

        if estimated_pos is not None:
            pos_x, pos_y, _ = estimated_pos
            x, y = self.get_canvas_coords(pos_x, pos_y)
            x1 = x - mark_radius
            y1 = y - mark_radius
            x2 = x + mark_radius
            y2 = y + mark_radius
            self.room_canvas.create_oval(x1, y1,
                                         x2, y2,
                                         fill="",
                                         outline=ESTIMATED_POS_COLOR,
                                         width=5,
                                         tags=(ESTIMATED_POS_TAG,))

        self.room_canvas.pack()

if __name__ == '__main__':
    times_per_second = 10
    milliseconds_per_update = int(1000 / times_per_second)

    root = tkinter.Tk()
    W = TKWorld(root, -495, 495, -495, 495, resolution=10)
    W.add_obs(Box(-245, -205, 205, 245))
    W.add_obs(Box(205, 245, 205, 245))
    W.add_obs(Box(-245, -205, -245, -205))
    W.add_obs(Box(205, 245, -245, -205))

    bot = CircleBot(10, 0, 0, 0)
    W.add_ent(bot)

    bot_control = SimBotControl(W, bot, arc_degrees = 5)

    options = {}
    options[slam.COLLISION_MAP_SCALE] = 30
    options[slam.COLLISION_MAP_MAX_DISTANCE] = 500
    options[slam.EKF_ODOMETRY_NOISE] = 0.01
    options[slam.EKF_RANGE_NOISE] = 12
    options[slam.EKF_BEARING_NOISE] = radians(1)
    options[slam.EKF_INNOVATION_LAMBDA] = 0.5
    options[slam.EKF_LANDMARK_THRESHOLD] = 7
    options[slam.SPIKE_THRESHOLD] = 400
    options[slam.RANSAC_SAMPLES] = 4
    options[slam.RANSAC_RANGE] = radians(90)
    options[slam.RANSAC_ERROR] = 10
    options[slam.RANSAC_CONSENSUS] = 8
    slam_instance = slam.Slam(bot_control, option_dictionary=options)
    explorer = Explorer(slam_instance)

    def update():
        start = datetime.now()
        W.display(collision_map = slam_instance.get_collision_map().map,
                  estimated_pos = slam_instance.get_estimated_position())
        explorer.step(debug=False)
        miliseconds_taken = (datetime.now() - start).total_seconds() * 1000

        if miliseconds_taken >= milliseconds_per_update:
            print("Warning, SLAM processing took too long: {}ms!".format(miliseconds_taken))
            root.after(0, update)
        else:
            root.after(int(milliseconds_per_update - miliseconds_taken), update)

    W.display()
    root.after(milliseconds_per_update, update)
    root.mainloop()

    f = open(os.path.join(".", "results", "tk_sim_collision_map.txt"), "w")
    f.write(str(slam_instance.get_collision_map()))
    f.close()
