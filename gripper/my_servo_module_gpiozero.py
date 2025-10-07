import serial, time

pico = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

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
