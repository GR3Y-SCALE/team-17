#!/usr/bin/env python3

import os,sys,time,math
from mobility.drive_system import DriveSystem
from navigation.NavClass import NavClass
from vision.cam_goodge_again import VisionSystem
from enum import Enum, auto

nav = NavClass(FOV=140, width=0.16)
robot = DriveSystem()
vision = VisionSystem()

class robot_state(Enum):
    DEBUGGING = auto()
    APPROACH_PICKING_STATION = auto()
    COLLECT_ITEM = auto()
    FIND_ISLE = auto()
    DESPOSIT_ITEM = auto()
    RETURN_TO_PICKING_STATION = auto()
robot_navigation_state = robot_state.DEBUGGING

class collection_state(Enum):
    ENTER_RAMP_BAY = auto()
    APPROACHING_ITEM = auto()
    PICKING_UP_ITEM = auto()
robot_collection_state = collection_state.HEADING_TO_CORRECT_BAY


def go_to_landmark(object_lambda, distance, speed, debug=False):
    '''
    Drives the robot to a landmark in the warehouse using potential field, handles the event of object not being visible.

    Args:
        pass lambda function of getter for vision object
        distance: distance from the object the robot will stop at
        speed: forward velocity of robot when going to landmark.
    '''

    # To Mozz
    # Switch case for different landmark types
    # to methods, it could be in the VisionSystem class itself
    # or you could have it as a parameter to this function.
    # Love Dan
    vision.UpdateObjects()
    landmark = object_lambda()

    if landmark is None:
        if debug: print("Landmark not visible")
        while landmark is None:
            if debug: print("Searching...")
            robot.SetTargetVelocities(0.0, -0.1)

            vision.UpdateObjects()
            landmark = object_lambda()
            time.sleep(0.001)
    robot.SetTargetVelocities(0.0, 0.0)
    if debug: print("Found!")
    obs = vision.get_obstacles()
    while landmark[0] > distance and landmark[0] is not None:
        if debug: print("Going to landmark...")
        vision.UpdateObjects()
        landmark = object_lambda()

        nav_data = nav.calculate_goal_velocities(int(math.degrees(landmark[1])), None, True)
        robot.SetTargetVelocities(speed, nav_data['rotational_vel'])
        time.sleep(0.001)
        if debug: print('Distance to landmark: ' + str(round(landmark[0],2)))
    robot.SetTargetVelocities(0.0, 0.0)
    print("Complete!")


iteration = 0
dt = 0.1
last_time = time.time()

picking_station_order = [0, 1, 2]
shelf_number = [0, 3, 5]
bay_number = [0, 1, 0]
bay_height = [0, 1, 2]
shelf_number_for_deposit = [0,3,4]
# turn_padding = [11, -40, -50]
bay_depth = [0.05, 0.05, 0.05] # How far in the robot needs to go for each shelf, partly redundant

def main():
    now = time.time()
    elapsed = now - last_time
    while elapsed < dt:
        time.sleep(dt - elapsed)
        continue
    last_time = now 

    try:
        robot.turn_degrees(90, 180)
        while True:
            # Update camera objects
            match robot_navigation_state:
                case robot_state.DEBUGGING:
                    vision.UpdateObjects()
                    time.sleep(0.1)

                case robot_state.APPROACH_PICKING_STATION:
                    # Use first shelf to navigate to picking station and turn to face the correct marker
                    robot_navigation_state = robot_state.COLLECT_ITEM
                case robot_state.COLLECT_ITEM:
                    print("Entering ramp")
                    payload.arm_position('max')
                    payload.open_claw()
                    go_to_landmark(picking_station_marker[picking_station_order[iteration]], 0.3, 0.02)
                    payload.arm_position('min')
                    go_to_landmark(picking_station_marker[picking_station_order[iteration]], 0.05, 0.01)
                    payload.close_claw()
                    robot.drive_distance(-0.5) # Maybe be more precise when exiting ramp, relying less on dead reckoning
                    robot_navigation_state = robot_state.FIND_ISLE
                case robot_state.FIND_ISLE:
                    
                    # search from left to right to find correct shelf
                    # use potential field to drive to correct shelf, possibly modifying the field so it drives alongside it
                    # once at correct shelf, turn until row marker is visible
                    # drive to row marker and stop at set distance
                    state = robot_state.DESPOSIT_ITEM
                case robot_state.DESPOSIT_ITEM:
                    # update camera
                    # turn to face shelf
                    # drive to shelf and stop at set distance
                    # deposit item
                    state = robot_state.RETURN_TO_PICKING_STATION
                case robot_state.RETURN_TO_PICKING_STATION:
                    # update camera
                    # turn to find row marker, then turn 180 degrees from it
                    # leave shelf area
                    # search from left to right to find picking station
                    # use potential field to drive to picking station, possibly modifying the field so it drives alongside it
                    # once at picking station, turn until bay marker is visible
                    iteration += 1
                    print(f"Completed {iteration} full iterations")
                    state = robot_state.COLLECT_ITEM

            
    except KeyboardInterrupt:
        print("Exiting")
        robot.stop_all()



if __name__ == "__main__":
    main()
