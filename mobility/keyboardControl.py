# -*- coding:utf-8 -*-

from __future__ import print_function
import sys
import os
from pynput import keyboard
from pynput.keyboard import Key
sys.path.append("../")

import time

from mobility.drive_system import DriveSystem

robot = DriveSystem()



def handRPM():
    import math
    from time import sleep, monotonic
    poll_dt = 0.01
    t0 = monotonic()
    sL = sR = 0.0
    t_prev = monotonic()
    s_last_change_t = t_prev
    dbg_t_last = t_prev
    while True:
        sleep(poll_dt)
        t_now = monotonic()
        dt = t_now - t_prev
        t_prev = t_now


def on_key_release(key):
    robot.stop_all()

def on_key_pressed(key):
    global duty
    global lDuty

    if key == Key.right:
        robot.set_target_velocities(0.0,50)

    elif key == Key.left:
        robot.set_target_velocities(0.0,-50)


    elif key == Key.up:
        robot.set_target_velocities(0.5,0.0)


    elif key == Key.down:
        robot.set_target_velocities(-0.5,0.0)

    elif key == Key.esc:
        robot.stop_all()
    
    

if __name__ == "__main__":
    print("Starting...")

    with keyboard.Listener(on_press=on_key_pressed, on_release=on_key_release) as listener:
        listener.join()


    