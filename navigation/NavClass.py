from navigation import state_machine as sm
from navigation import NavMath as navmath

import numpy as np
import math 
import time

debug = True

class NavClass:

    def __init__(self, FOV):
        self.FOV = FOV
        self.attractive_field = np.zeros(FOV)
        self.repulsive_field = np.zeros(FOV)

        self.forward_vel = 0
        self.rot_vel = 0

        self.wicked_sm = sm.stateMachine()

    def update(self):
        self.wicked_sm.update_state()

        # compute potential fields from camera view
        self.compute_attractor_field() # Feed in useful data

    def compute_attractor_field(self, goal_deg): # feed in angle of goal
        self.attraction_field[goal_deg]
        gradient = 1 / 30
        for angle in range(0, int((CAMERA_FOV/2 + 1))):
            self.attractive_field[navmath.clip]

        # get OM from camera
        # get GD from camera
        # Motor heading map generated from (GD + OM)

        # get maximum peak to find path to travel, noting the peak is the angle of the motors and the magnitude is speed

