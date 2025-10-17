from navigation import state_machine as sm
from navigation import NavMath as navmath
from navigation.range_finder.PiicoDev_VL53L1X import PiicoDev_VL53L1X

import numpy as np
import math
import time
import matplotlib.pyplot as plt

debug = False
class NavClass:

    def __init__(self, FOV, width=0.16, range_finder=False, state_machine=None, robot=None):
        self.FOV = FOV
        self.width = width
        self.attractive_field = np.zeros(FOV + 1)
        self.repulsive_field = np.zeros(FOV + 1)

        if range_finder:
            self.distSensor = PiicoDev_VL53L1X()

        self.forward_vel = 0
        self.rot_vel = 0
        # Optional state machine integration
        if state_machine is not None:
            self.state_machine = state_machine
        elif robot is not None:
            self.state_machine = sm.StateMachine(robot)
        else:
            self.state_machine = None
        print("NavClass initialised successfully.")

    def update(self, event=None):
        if self.state_machine is not None:
            self.state_machine.update_state(event)

        # compute potential fields from camera view
        # You would likely call calculate_goal_velocities from here
        # with the current goal and detected obstacles.
    def get_range_finder_distance(self):
        return self.distSensor.read()

    def compute_attractive_field(self, goal_deg): # feed in angle of goal
        attractive_field = np.zeros(self.FOV + 1)
        attractive_field[navmath.clip_deg_fov(goal_deg, self.FOV)] = 1 # Highest potential is where the goal is
        gradient = 1 / 30
        for angle in range(0, int((self.FOV / 2 + 1))):
            attractive_field[navmath.clip_deg_fov(goal_deg - angle, self.FOV)] = 1 - angle * gradient
            attractive_field[navmath.clip_deg_fov(goal_deg + angle, self.FOV)] = 1 - angle * gradient
        return attractive_field

    def compute_repulsive_field(self, obstacles):
        min_obstacle_dist = 8 # minimum distance to object to consider
        repulsive_field = np.zeros(self.FOV + 1)
        if obstacles is None or len(obstacles) == 0:
            return np.zeros(self.FOV + 1)
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

    def calculate_goal_velocities(self, goal_deg, obstacles=None, debug=False):
        MAX_ROBOT_VEL = 0.05
        MAX_ROBOT_ROT = 0.075
        GOAL_P = 0.01 # Proportional bias
        ROTATIONAL_BIAS = 0.05
        CAMERA_FOV = self.FOV # Use the FOV from the class

        nav_state = {}
        # Call methods using self
        goal_deg_reversed = int((CAMERA_FOV / 2) - goal_deg)
        nav_state['attractive_field'] = self.compute_attractive_field(goal_deg_reversed) # reversed heading to match API

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
        if debug:
            plt.figure(2)
            plt.clf()

            plt.subplot(3,1,1)
            plt.plot(nav_state['attractive_field'], label='Attractive Field')

            plt.subplot(3,1,2)
            plt.plot(nav_state['repulsive_field'], label='Repulsive Field', color='r')

            plt.subplot(3,1,3)
            plt.plot(nav_state['residual_map'], label='Residual Map', color='g')
            plt.axvline(x=heading_angle, color='k', linestyle='--', label='Heading Angle')
            plt.legend()
            plt.pause(0.001)
            plt.draw()

        return nav_state
