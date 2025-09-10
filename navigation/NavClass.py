from navigation import state_machine as sm
from navigation import NavMath as navmath

import numpy as np
import math 
import time

debug = True

class NavClass:

    def __init__(self, FOV, width=0.16):
        self.FOV = FOV
        self.width = width
        self.attractive_field = np.zeros(FOV)
        self.repulsive_field = np.zeros(FOV)

        self.forward_vel = 0
        self.rot_vel = 0

        self.wicked_sm = sm.stateMachine()

    def update(self):
        self.wicked_sm.update_state()

        # compute potential fields from camera view
        #TODO find goal in degrees
        self.compute_attractor_field(goal_deg)

    def compute_attractor_field(self, goal_deg): # feed in angle of goal
        self.attractive_field = np.zeros(self.FOV+1)
        self.attractive_field[goal_deg] = 1 # Highest potential is where the goal is
        gradient = 1 / 30
        for angle in range(0, int((self.FOV/2 + 1))):
            self.attractive_field[navmath.clip_deg_fov(goal_deg - angle, self.FOV)] = 1 - angle * gradient
            self.attractive_field[navmath.clip_deg_fov(goal_deg + angle, self.FOV)] = 1 - angle * gradient
        return self.attractive_field
    
    def compute_repulsive_field(self, obstacles):
        min_obstacle_dist = 0.25 # minimum distance to object to consider
        repulsive_field =np.zeros(self.FOV+1)

        if obstacles:
            for obs in obstacles:
                obs_dist = obs.distance_to_robot
                obs_deg = obs.degree

                if obs_dist < min_obstacle_dist:
                    obs_width_rad = 2*math.atan(self.width / obs_dist)
                    obs_width_deg = int(obs_width_rad * (180/math.pi))

                    obs_strength = max(0, 1 - min(1, obs_dist - self.width*2))

                    repulsive_field[obs_deg] = obs_strength # asserts the strongest repulsion happens at the centre of the obstacle

                    for angle in range(1, obs_width_deg+1):
                        repulsive_field[navmath.clip_deg_fov(obs_deg - angle, self.FOV)] = max(repulsive_field[navmath.clip_deg_fov(obs_deg - angle, self.FOV)], obs_strength)
                        repulsive_field[navmath.clip_deg_fov(obs_deg + angle, self.FOV)] = max(repulsive_field[navmath.clip_deg_fov(obs_deg + angle, self.FOV)], obs_strength)
        return repulsive_field

    def calculate_goal_velocities