#!/usr/bin/env python3
"""
Tiny keyboard tele‑op for the DFRobot DC‑Motor I2C board.
W/S = forward/reverse • A/D = gentle turn • Space = stop • X or Ctrl‑C = quit
"""
import os, sys, tty, termios, signal
from DFRobot_RaspberryPi_DC_Motor import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board

# ── Board setup ────────────────────────────────────────────────────────────
board = Board(1 if THIS_BOARD_TYPE else 7, 0x10)
while board.begin() != board.STA_OK:
    pass                     # wait until the board replies
FWD, REV = board.CW, board.CCW
SPEED = 100                  # duty‑cycle percentage

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

    try:
        while True:
            k = read_key()
            if   k == "w":  set_motor(board.M1,  SPEED); set_motor(board.M2,  SPEED)
            elif k == "s":  set_motor(board.M1, -SPEED); set_motor(board.M2, -SPEED)
            elif k == "a":  set_motor(board.M1,     0);  set_motor(board.M2,  SPEED)
            elif k == "d":  set_motor(board.M1,  SPEED); set_motor(board.M2,     0)
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