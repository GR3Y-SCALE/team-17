import serial
import time

pico = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(1)

commands = ["L0", "G0", "L3", "G1"]

for cmd in commands:
    pico.write(f"{cmd}\n".encode())
    time.sleep(0.5)
    while pico.in_waiting:
        print(pico.readline().decode().strip())

pico.close()
