from gpiozero import Servo
from time import sleep
import numpy as np

# FS90R on GPIO19
servo = Servo(19, min_pulse_width=0.0010, max_pulse_width=0.0020)

# Create smooth sweep values between -0.3 and +0.3
sweep_values = np.linspace(-0.3, 0.3, 30)  # 30 steps for smooth motion

try:
    while True:
        # Sweep left to right
        for val in sweep_values:
            servo.value = val
            sleep(0.05)  # adjust speed: smaller = slower

        # Sweep right to left
        for val in reversed(sweep_values):
            servo.value = val
            sleep(0.05)

except KeyboardInterrupt:
    servo.mid()  # stop servo on exit
    print("Stopped cleanly")
