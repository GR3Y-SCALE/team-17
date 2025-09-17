#!/usr/bin/env python3
"""
Tiny keyboard tele‑op for the DFRobot DC‑Motor I2C board.
W/S = forward/reverse • A/D = gentle turn • Space = stop • X or Ctrl‑C = quit
"""
import os, sys, tty, termios, signal
from DFRobot_RaspberryPi_DC_Motor import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board

# ── Board setup ────────────────────────────────────────────────────────────
# board = Board(1 if THIS_BOARD_TYPE else 7, 0x10)   # original line
board = Board(1, 0x10)  # force Raspberry Pi to use I²C bus 1
while board.begin() != board.STA_OK:
    pass                     # wait until the board replies
FWD, REV = board.CW, board.CCW

SPEED_FAST = 100
SPEED_SLOW = 30
current_speed = SPEED_FAST                 # duty‑cycle percentage

# ── Tiny helpers ───────────────────────────────────────────────────────────
def set_motor(motor, value: int):
    """ value ∈ [-100,100] : sign = direction, abs = duty """
    value = max(-100, min(100, value))
    dir_ = FWD if value >= 0 else REV
    board.motor_movement([motor], dir_, abs(value))

def stop_all():
    for m in (board.M1, board.M2):
        board.motor_movement([m], FWD, 0)

def read_key():
    c = os.read(sys.stdin.fileno(), 1)
    if not c:          # EOF
        return ""
    if c == b"\x03":   # Ctrl‑C
        return "quit"
    return c.decode().lower()

# ── Main loop ──────────────────────────────────────────────────────────────
def main():
    old = termios.tcgetattr(sys.stdin.fileno())
    tty.setraw(sys.stdin.fileno())

    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    # VERY IMPORTANT INITIALISATION
    board.set_encoder_enable(board.ALL)
    board.set_encoder_reduction_ratio(board.ALL, 100)
    board.set_moter_pwm_frequency(1000)

    global current_speed  # will change this inside the loop

    try:
        while True:
            k = read_key()

            # --- NEW: speed presets ---
            if   k == "f":  # fast
                current_speed = SPEED_FAST
                print("Speed set to FAST =", current_speed)
                continue
            elif k == "g":  # gentle
                current_speed = SPEED_SLOW
                print("Speed set to GENTLE =", current_speed)
                continue

            # --- NEW: use current_speed instead of SPEED ---
            if   k == "w":  set_motor(board.M1,  current_speed); set_motor(board.M2, -current_speed)
            elif k == "s":  set_motor(board.M1, -current_speed); set_motor(board.M2,  current_speed)
            elif k == "a":  set_motor(board.M1,               0); set_motor(board.M2, -current_speed)
            elif k == "d":  set_motor(board.M1,  current_speed); set_motor(board.M2,               0)
            elif k == " ":  stop_all()
            elif k in {"x", "quit"}:
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop_all()
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)


if __name__ == "__main__":
    main()

