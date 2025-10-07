<<<<<<< HEAD
import serial, time
=======
from gpiozero import Servo
from time import sleep
>>>>>>> 8da5c64b2ce09dd64d29f10ae8054e25ee3f4131

pico = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

<<<<<<< HEAD
def move_lift(level):
    pico.write(f"L{level}\n".encode())

def open_gripper():
    pico.write(b"G1\n")

def close_gripper():
    pico.write(b"G0\n")

# Example sequence
move_lift(0)   # ground
close_gripper()
time.sleep(2)
move_lift(3)   # shelf 3
open_gripper()
=======
def open_gripper():
    print("Opening gripper...")
    servo.value = +0.5   # CCW (adjust speed/direction as needed)
    sleep(1.0)           # run for 1 second
    servo.value = 0      # stop

def close_gripper():
    print("Closing gripper...")
    servo.value = -0.5   # CW
    sleep(1.0)           # run for 1 second
    servo.value = 0      # stop

try:
    while True:
        open_gripper()
        sleep(2)
        close_gripper()
        sleep(2)

except KeyboardInterrupt:
    servo.value = 0  # stop servo on exit
    print("Stopped cleanly") 
>>>>>>> 8da5c64b2ce09dd64d29f10ae8054e25ee3f4131
