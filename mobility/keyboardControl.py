# -*- coding:utf-8 -*-

from __future__ import print_function
import sys
import os
from pynput import keyboard
from pynput.keyboard import Key
sys.path.append("../")

import time

from DFRobot_RaspberryPi_DC_Motor import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board


board = Board(1, 0x10)

# Global duty cycle variable
duty = 75
lDuty = 0.9*duty
duty_step = 2  # how much to increase/decrease with +/-

def board_detect():
    l = board.detecte()
    print("Board list conform:")
    print(l)

def print_board_status():
    if board.last_operate_status == board.STA_OK:
        print("board status: everything ok")
    elif board.last_operate_status == board.STA_ERR:
        print("board status: unexpected error")
    elif board.last_operate_status == board.STA_ERR_DEVICE_NOT_DETECTED:
        print("board status: device not detected")
    elif board.last_operate_status == board.STA_ERR_PARAMETER:
        print("board status: parameter error, last operate no effective")
    elif board.last_operate_status == board.STA_ERR_SOFT_VERSION:
        print("board status: unsupported board firmware version")










def handRPM():
    import math
    from time import sleep, monotonic
    poll_dt = 0.01
    board.set_encoder_enable([board.M1, board.M2])
    board.set_encoder_reduction_ratio([board.M1, board.M2], int(50))
    circ = math.pi*(30/1000)

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

        # encoders -> distance
        rpmL, rpmR = board.get_encoder_speed([board.M1, board.M2])
        revL = abs(rpmL) * dt / 60.0
        revR = abs(rpmR) * dt / 60.0

        sL += revL * circ
        sR += revR * circ
        s_enc  = 0.5 * (sL + sR)

        print("DEBUGGING:")
        print("revL", revL)
        print("revR", revR)
        print("sL", sL)
        print("sR", sR)
        print("s_enc", s_enc)

def on_key_release(key):
    board.motor_stop(board.ALL)

def on_key_pressed(key):
    global duty
    global lDuty

    if key == Key.right:
        print(f"Right key pressed at duty {duty}%")
        board.motor_movement([board.M1], board.CW, duty)
        board.motor_movement([board.M2], board.CW, duty)

    elif key == Key.left:
        print(f"Left key pressed at duty {duty}%")
        board.motor_movement([board.M1], board.CCW, duty)
        board.motor_movement([board.M2], board.CCW, duty)

    elif key == Key.up:
        print(f"Up key pressed at duty {duty}%")
        board.motor_movement([board.M1], board.CCW, duty)
        board.motor_movement([board.M2], board.CW, lDuty)

    elif key == Key.down:
        print(f"Down key pressed at duty {duty}%")
        board.motor_movement([board.M1], board.CW, duty)
        board.motor_movement([board.M2], board.CCW, lDuty)

    elif key == Key.esc:
        print("ESC pressed: stopping motors for 3s")
        board.motor_stop(board.ALL)
        time.sleep(3)
    
    

    # Adjust duty with + and -
    elif hasattr(key, 'char') and key.char == '+':
        duty = min(100, duty + duty_step)
        lDuty = 0.9*duty
        print(f"Duty increased to {duty}%")

    elif hasattr(key, 'char') and key.char == '-':
        duty = max(0, duty - duty_step)
        lDuty = 0.9*duty
        print(f"Duty decreased to {duty}%")

if __name__ == "__main__":
    print("Starting...")

    #initServos()
    #centre()
    board_detect()

    while board.begin() != board.STA_OK:
        print_board_status()
        print("board begin failed")
        time.sleep(2)

    print("board begin success")
    board.set_encoder_enable(board.ALL)
    board.set_encoder_reduction_ratio(board.ALL, 50)
    board.set_moter_pwm_frequency(2000)
    

    with keyboard.Listener(on_press=on_key_pressed, on_release=on_key_release) as listener:
        listener.join()


    