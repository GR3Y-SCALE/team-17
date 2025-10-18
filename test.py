from mobility.drive_system import DriveSystem
import time
robot = DriveSystem()
#robot.turn_degrees(90  ,180)
robot.drive_distance(-0.3, 0.2)
robot.turn_degrees(90, 90)

