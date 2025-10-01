from gpiozero import Servo
from time import sleep

# FS90R on GPIO19
servo = Servo(19, min_pulse_width=0.0010, max_pulse_width=0.0020)

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