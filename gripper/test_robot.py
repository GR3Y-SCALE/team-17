import time
import my_servo_module.py as servo   # import your servo code

print("=== TEST START ===")

# Test the gripper
print("Opening gripper...")
servo.open_gripper()
time.sleep(2)

print("Closing gripper for 3 seconds...")
servo.close_gripper_timed(3)
time.sleep(2)

# Test the lift arm
print("Moving lift to bottom shelf...")
servo.move_to("bottom_shelf")
time.sleep(2)

print("Moving lift to middle shelf...")
servo.move_to("middle_shelf")
time.sleep(2)

print("Moving lift to top shelf...")
servo.move_to("top_shelf")
time.sleep(2)

print("Returning lift to bottom shelf...")
servo.move_to("bottom_shelf")
time.sleep(2)

# Stop everything safely
servo.stop_servos()
print("=== TEST COMPLETE ===")
