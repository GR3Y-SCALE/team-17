#!/usr/bin/env python3

import os,sys,time
from mobility.drive_system import DriveSystem
from navigation.NavClass import NavClass

nav = NavClass(FOV=180, width=0.16)
robot = DriveSystem()

def main():

    try:
        robot.turn_degrees(90, 180)
        while True:
            print(robot.get_wheel_speeds())
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting")
        robot.stop_all()



if __name__ == "__main__":
    main()
