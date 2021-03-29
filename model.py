import numpy as np

class Robot():
    def __init__(self, init_pos = [0,0], init_vel = [0,0]):
        self.x = init_pos[0]
        self.y = init_pos[1]
        self.vx = init_vel[0]
        self.vy = init_vel[1]

    def move_me(self, step):
        self.x += step[0]
        self.y += step[1]

    def get_pos(self):
        return [self.x, self.y]

    def get_vel(self):
        return [self.vx, self.vy]

    def update_vel(self, vel):
        self.vx = vel[0]
        self.vy = vel[1]
