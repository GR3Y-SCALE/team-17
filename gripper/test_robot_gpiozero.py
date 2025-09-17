import time
import my_servo_module_gpiozero as servo

print("=== TEST START ===")

# Test gripper
print("Opening gripper...")
servo.open_gripper()
time.sleep(1)

print("Closing gripper for 3 seconds...")
servo.close_gripper_timed(3)
time.sleep(1)

# Test lift arm
print("Moving lift to bottom shelf...")
servo.move_to("bottom_shelf")
time.sleep(1)

print("Moving lift to middle shelf...")
servo.move_to("middle_shelf")
time.sleep(1)

print("Moving lift to top shelf...")
servo.move_to("top_shelf")
time.sleep(1)

# Return lift to bottom
servo.move_to("bottom_shelf")
time.sleep(1)

# Stop everything
servo.stop_servos()
print("=== TEST COMPLETE ===")
