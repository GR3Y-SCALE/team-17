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
state = robot_state.COLLECT_ITEM

iteration = 0

def main():

    try:
        robot.turn_degrees(90, 180)
        while True:
            # Update camera objects
            match state:
                case robot_state.COLLECT_ITEM:
                    # update camera objects
                    # drive to correct collection bay and stop at set distance
                    # drive slowly to item and stop at set distance
                    # pick up item
                    state = robot_state.FIND_SHELF
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
