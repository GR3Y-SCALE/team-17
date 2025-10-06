import os, sys, time
from DFRobot_RaspberryPi_DC_Motor import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board


class DriveSystem:
    def __init__(self):
        self.board = Board(1, 0x10)  # force Raspberry Pi to use I²C bus 1
        while self.board.begin() != self.board.STA_OK:
            pass
        print("Motor driver initialised and reporting OK")
        self.board.set_encoder_enable(self.board.ALL)
        self.board.set_encoder_reduction_ratio(self.board.ALL, 100)
        self.board.set_moter_pwm_frequency(1000)

    def stop_all(self):
        self.board.motor_stop(self.board.ALL)