from gpiozero import Servo, Device
from time import sleep

# -------------------------
# Safe cleanup first
# -------------------------
try:
    Device.pin_factory.close()
except:
    pass  # ignore if nothing to close

# -------------------------
# GPIO Pins
# -------------------------
GRIPPER_SERVO_PIN = 12  # GPIO for gripper
LIFT_SERVO_PIN    = 13  # GPIO for lift arm

# -------------------------
# Initialize servos
# -------------------------
gripper = Servo(GRIPPER_SERVO_PIN)
lift = Servo(LIFT_SERVO_PIN)

# -------------------------
# Helper: map 0-180° to -1..1
# -------------------------
def angle_to_ratio(angle):
    return (angle / 90.0) - 1  # 0° -> -1, 90° -> 0, 180° -> 1

# -------------------------
# Gripper positions (replace with measured angles)
# -------------------------
GRIPPER_POSITIONS = {
    "open": 20,    # temporary value
    "closed": 120  # temporary value
}

def open_gripper():
    ratio = angle_to_ratio(GRIPPER_POSITIONS["open"])
    gripper.value = ratio
    sleep(1)

def close_gripper_timed(duration=3):
    ratio = angle_to_ratio(GRIPPER_POSITIONS["closed"])
    gripper.value = ratio
    sleep(duration)

# -------------------------
# Lift positions (replace with measured angles)
# -------------------------
LIFT_POSITIONS = {
    "bottom_shelf": 0,
    "middle_shelf": 90,
    "top_shelf": 180
}

current_lift_angle = 90  # starting guess

def move_to(position):
    global current_lift_angle
    if position not in LIFT_POSITIONS:
        raise ValueError(f"Invalid position '{position}'")
    
    target_angle = LIFT_POSITIONS[position]

    # Smooth move
    steps = 20
    start_ratio = angle_to_ratio(current_lift_angle)
    end_ratio = angle_to_ratio(target_angle)
    for i in range(steps + 1):
        ratio = start_ratio + (end_ratio - start_ratio) * i / steps
        lift.value = ratio
        sleep(0.1)  # adjust speed here
    current_lift_angle = target_angle

# -------------------------
# Stop servos safely
# -------------------------
def stop_servos():
    gripper.detach()
    lift.detach()
    try:
        Device.pin_factory.close()
    except:
        pass
