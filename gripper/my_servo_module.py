from gpiozero import Servo
from time import sleep

# -------------------------
# Setup
# -------------------------
GRIPPER_SERVO_PIN = 18  # GPIO for gripper
LIFT_SERVO_PIN    = 19  # GPIO for lift arm

gripper = Servo(GRIPPER_SERVO_PIN)
lift = Servo(LIFT_SERVO_PIN)

# -------------------------
# Mapping helper (-1 to 1)
# -------------------------
def angle_to_ratio(angle):
    """
    Map 0-180 degrees to -1 to 1 for gpiozero Servo
    """
    return (angle / 90.0) - 1  # 0° -> -1, 90° -> 0, 180° -> 1

# -------------------------
# Gripper positions
# -------------------------
GRIPPER_POSITIONS = {
    "open": XXXX,    # degrees
    "closed": XXXX   # degrees
}

def open_gripper():
    ratio = angle_to_ratio(GRIPPER_POSITIONS["open"])
    gripper.value = ratio
    sleep(1)  # wait for servo to move

def close_gripper_timed(duration=3):
    ratio = angle_to_ratio(GRIPPER_POSITIONS["closed"])
    gripper.value = ratio
    sleep(duration)

# -------------------------
# Lift positions
# -------------------------
LIFT_POSITIONS = {
    "bottom_shelf": XXXX,
    "middle_shelf": XXXX,
    "top_shelf": XXXX
}

current_lift_angle = 90  # starting guess

def move_to(position):
    global current_lift_angle
    if position not in LIFT_POSITIONS:
        raise ValueError(f"Invalid position '{position}'")
    
    target_angle = LIFT_POSITIONS[position]
    
    # Smooth move (simple incremental)
    steps = 20
    start_ratio = angle_to_ratio(current_lift_angle)
    end_ratio = angle_to_ratio(target_angle)
    for i in range(steps + 1):
        ratio = start_ratio + (end_ratio - start_ratio) * i / steps
        lift.value = ratio
        sleep(0.05)  # adjust speed
    current_lift_angle = target_angle

# -------------------------
# Stop servos (set to neutral)
# -------------------------
def stop_servos():
    gripper.detach()
    lift.detach()
