from mobility.drive_system import DriveSystem
import time
robot = DriveSystem()
robot.turn_degrees(90, 180)
time.sleep(1)
robot.drive_distance(50,0.15)
