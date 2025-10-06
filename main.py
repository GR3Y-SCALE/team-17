#!/usr/bin/env python3

import os,sys,time
from mobility.drive_system import DriveSystem
from navigation.NavClass import NavClass

nav = NavClass(FOV=180, width=0.16)
robot = DriveSystem()

def main():
    print(robot.get_wheel_speeds())
    time.sleep(1)



if __name__ == "__main__":
    main()
