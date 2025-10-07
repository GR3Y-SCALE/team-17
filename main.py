#!/usr/bin/env python3

import os,sys,time
from mobility.drive_system import DriveSystem
from navigation.NavClass import NavClass
from enum import Enum, auto

nav = NavClass(FOV=180, width=0.16)
robot = DriveSystem()

class robot_state(Enum):
    COLLECT_ITEM = auto()
    FIND_SHELF = auto()
    DESPOSIT_ITEM = auto()
    RETURN_TO_PICKING_STATION = auto()
robot_navigation_state = robot_state.COLLECT_ITEM

class collection_state(Enum):
    HEADING_TO_CORRECT_BAY = auto()
    APPROACHING_ITEM = auto()
    PICKING_UP_ITEM = auto()
robot_collection_state = collection_state.HEADING_TO_CORRECT_BAY

iteration = 0
dt = 0.1
last_time = time.time()

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
                case robot_state.COLLECT_ITEM:
                    # update camera objects
                    match robot_collection_state:
                        case collection_state.HEADING_TO_CORRECT_BAY:
                            # turn left
                            if not bay_marker[iteration]:
                                print("Error: No bay marker found")
                                # turn right until the correct marker is found
                            else:
                                # nav_data = nav.calculate_goal_velocities(bay_maker[iteration][1]) This feeds in the heading of the bay marker
                                # robot.set_target_velocities(0.01, nav_data['rotational_velocity'])

                            robot_collection_state = collection_state.APPROACHING_ITEM
                        case collection_state.APPROACHING_ITEM:
                            # blah blah
                            robot_collection_state = collection_state.PICKING_UP_ITEM
                        case collection_state.PICKING_UP_ITEM:
                            # blah blah
                            robot_collection_state = collection_state.HEADING_TO_CORRECT_BAY
                            robot_navigation_state = robot_state.FIND_SHELF


                    # drive slowly to item and stop at set distance
                    # pick up item
                case robot_state.FIND_SHELF:
                    # update camera
                    # Turn 180 degrees from marker
                    # Leave picking bay
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
