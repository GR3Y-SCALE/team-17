from navigation import state_machine as sm

import numpy as np
import math 
import time

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
        self.computePotentialField() # Feed in useful data

    def computePotentialField():
        # get OM from camera
        # get GD from camera
        # Motor heading map generated from (GD + OM)

        # get maximum peak to find path to travel, noting the peak is the angle of the motors and the magnitude is speed