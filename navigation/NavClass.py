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
        self.attractive_field = np.zeros(FOV + 1)
        self.repulsive_field = np.zeros(FOV + 1)

        self.forward_vel = 0
        self.rot_vel = 0

        self.wicked_sm = sm.State()

    def update(self):
        self.wicked_sm.update_state()

        # compute potential fields from camera view
        # You would likely call calculate_goal_velocities from here
        # with the current goal and detected obstacles.

    def compute_attractive_field(self, goal_deg): # feed in angle of goal
        attractive_field = np.zeros(self.FOV + 1)
        attractive_field[navmath.clip_deg_fov(goal_deg, self.FOV)] = 1 # Highest potential is where the goal is
        gradient = 1 / 30
        for angle in range(0, int((self.FOV / 2 + 1))):
            attractive_field[navmath.clip_deg_fov(goal_deg - angle, self.FOV)] = max(0, 1 - angle * gradient)
            attractive_field[navmath.clip_deg_fov(goal_deg + angle, self.FOV)] = 1 - angle * gradient
        return attractive_field

    def compute_repulsive_field(self, obstacles):
        min_obstacle_dist = 0.25 # minimum distance to object to consider
        repulsive_field = np.zeros(self.FOV + 1)
        for obs in obstacles:
            obs_dist = obs.distance_to_robot
            obs_deg = obs.degree

            if obs_dist < min_obstacle_dist:
                obs_width_rad = 2 * math.atan(self.width / obs_dist)
                obs_width_deg = int(obs_width_rad * (180 / math.pi))

                # Corrected strength calculation
                obs_strength = 1.0 - (obs_dist / min_obstacle_dist)

                repulsive_field[obs_deg] = obs_strength # asserts the strongest repulsion happens at the centre of the obstacle

                for angle in range(1, obs_width_deg + 1):
                    left_idx = navmath.clip_deg_fov(obs_deg - angle, self.FOV)
                    right_idx = navmath.clip_deg_fov(obs_deg + angle, self.FOV)
                    repulsive_field[left_idx] = max(repulsive_field[left_idx], obs_strength)
                    repulsive_field[right_idx] = max(repulsive_field[right_idx], obs_strength)
        
        return repulsive_field

    def calculate_goal_velocities(self, goal_deg, obstacles):
        MAX_ROBOT_VEL = 0.05
        MAX_ROBOT_ROT = 0.2
        GOAL_P = 0.05 # Proportional bias
        ROTATIONAL_BIAS = 0.05
        CAMERA_FOV = self.FOV # Use the FOV from the class

        nav_state = {}
        # Call methods using self
        nav_state['attractive_field'] = self.compute_attractive_field(goal_deg)

        if obstacles is None or len(obstacles) == 0:
            nav_state['repulsive_field'] = np.zeros(CAMERA_FOV + 1)
        else:
            nav_state['repulsive_field'] = self.compute_repulsive_field(obstacles)

        nav_state['residual_map'] = np.maximum(0, nav_state['attractive_field'] - nav_state['repulsive_field'])

        # get heading
        heading_angle = np.argmax(nav_state['residual_map'])
        goal_error = heading_angle - CAMERA_FOV / 2

        # balance differential drive
        nav_state['rotational_vel'] = min(MAX_ROBOT_ROT, max(-MAX_ROBOT_ROT, goal_error * GOAL_P))
        
        # Use the correct variable name for rotational velocity
        nav_state['forward_vel'] = MAX_ROBOT_VEL * (1.0 - ROTATIONAL_BIAS * abs(nav_state['rotational_vel']) / MAX_ROBOT_ROT)

        # Update class members
        self.rot_vel = nav_state['rotational_vel']
        self.forward_vel = nav_state['forward_vel']

        return nav_state