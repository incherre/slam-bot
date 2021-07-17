'''Robot sim with a nicer display.'''
from sim_framework import *
from math import radians
import tkinter

BACKGROUND_COLOR = 'grey60'
ENTITY_COLOR = 'RoyalBlue1'
OBSTACLE_COLOR = 'black'
ENTITY_TAG = 'entity'

class TKWorld(World):
    '''A world that will display via tkinter instead of ascii.'''
    def __init__(self, root, x_min, x_max, y_min, y_max, resolution=2, max_dist=10000, collision_delta_theta=1):
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

    def display(self):
        '''Displays the environment, by default just as a character array.'''
        try:
            self.room_canvas.delete(ENTITY_TAG)
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

        self.room_canvas.pack()

if __name__ == '__main__':
    root = tkinter.Tk()
    W = TKWorld(root, -500, 500, -500, 500)
    W.add_obs(Box(-500, -250, 250, 500))
    W.add_obs(Box(-450, -200, 200, 450))
    W.add_obs(Box(-400, -150, 150, 400))
    W.add_obs(Box(-350, -100, 100, 350))

    bot = CircleBot(100, 0, 0, 0)
    W.add_ent(bot)

    theta = radians(0)
    def update():
        root.after(int(1000 / 60), update)

        global theta
        W.display()
        theta -= radians(0.2)
        if W.move_ent(bot, 5, theta):
            theta -= radians(360 * 1.618)
            theta = theta % radians(360)

    root.after(int(1000 / 60), update)
    root.mainloop()
