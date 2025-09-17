import pigpio
import time

# -------------------------
# Setup
# -------------------------
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Cannot connect to pigpiod. Did you start it?")

GRIPPER_SERVO = 12    # Ask moss for the correct gpio pin should be these mabeyor gpio 18 gpio 19
LIFT_SERVO    = 13    # GPIO pin for lift arm

# -------------------------
# Helper: angle -> pulsewidth
# -------------------------
def set_servo_angle(gpio, angle):
    """Directly set servo to angle (0–180)."""
    pulse = 500 + (angle / 180.0) * 2000
    pi.set_servo_pulsewidth(gpio, pulse)

def smooth_move(gpio, start, end):
    """
    Smoothly move a servo from start to end angle (~4.5 s for 180°).
    """
    step = 1
    delay = 0.025  # slower, smooth movement

    if start < end:
        rng = range(start, end + 1, step)
    else:
        rng = range(start, end - 1, -step)

    for angle in rng:
        set_servo_angle(gpio, angle)
        time.sleep(delay)

# -------------------------
# Gripper positions (Servo 1)
# Replace XXXX with measured angles
# -------------------------
GRIPPER_POSITIONS = {
    "open": XXXX,
    "closed": XXXX
}

def open_gripper():
    """Open the gripper smoothly."""
    smooth_move(GRIPPER_SERVO, GRIPPER_POSITIONS["closed"], GRIPPER_POSITIONS["open"])

def close_gripper_timed(duration=3):
    """
    Close the gripper toward the closed angle for a fixed time (seconds),
    then stop the servo.
    """
    target_angle = GRIPPER_POSITIONS["closed"]
    
    # Move toward closed smoothly
    smooth_move(GRIPPER_SERVO, GRIPPER_POSITIONS["open"], target_angle)
    
    # Hold position for the duration
    time.sleep(duration)
    
    # Stop sending PWM (servo holds last position)
    pi.set_servo_pulsewidth(GRIPPER_SERVO, 0)

# -------------------------
# Lift positions (Servo 2)
# Replace XXXX with measured angles
# -------------------------
LIFT_POSITIONS = {
    "bottom_shelf": XXXX,
    "middle_shelf": XXXX,
    "top_shelf": XXXX
}

current_lift_angle = 90  # initial guess, update after first move

def move_to(position):
    """Move lift arm to one of three preset positions smoothly."""
    global current_lift_angle
    if position not in LIFT_POSITIONS:
        raise ValueError(f"Invalid position '{position}'. Choose from {list(LIFT_POSITIONS.keys())}")

    target_angle = LIFT_POSITIONS[position]
    smooth_move(LIFT_SERVO, current_lift_angle, target_angle)
    current_lift_angle = target_angle

# -------------------------
# Cleanup
# -------------------------
def stop_servos():
    """Stop sending PWM signals and release resources."""
    pi.set_servo_pulsewidth(GRIPPER_SERVO, 0)
    pi.set_servo_pulsewidth(LIFT_SERVO, 0)
    pi.stop()