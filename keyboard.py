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
SPEED_SLOW = 40
current_speed = SPEED_FAST                 # duty‑cycle percentage

SPIN_SPEED = SPEED_SLOW
is_spinning = False   # toggled by 'r'


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

def spin_in_place(clockwise: bool = True):
    """
    Command an in-place spin at SPIN_SPEED using current motor mounting.
    'clockwise=True' spins one way; False reverses it.
    """
    # With current mapping: forward uses (+ on M1, - on M2).
    # For an in-place spin we drive BOTH motors the same sign.
    v = SPIN_SPEED if clockwise else -SPIN_SPEED
    set_motor(board.M1, v)
    set_motor(board.M2, v)

# ── Main loop ──────────────────────────────────────────────────────────────
def main():
    old = termios.tcgetattr(sys.stdin.fileno())
    tty.setraw(sys.stdin.fileno())

    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    # VERY IMPORTANT INITIALISATION
    board.set_encoder_enable(board.ALL)
    board.set_encoder_reduction_ratio(board.ALL, 100)
    board.set_moter_pwm_frequency(1000)

    # === NEW SPIN MODE: uses globals set elsewhere (SPIN_SPEED, is_spinning, spin_in_place) ===
    global current_speed, is_spinning
    # === END NEW SPIN MODE ===

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

            # === NEW SPIN MODE: toggle with 'r' (spin in place at slow speed) ===
            elif k == "r":
                if not is_spinning:
                    is_spinning = True
                    current_speed = SPEED_SLOW  # ensure slow while spinning
                    spin_in_place(clockwise=True)  # set False for opposite direction
                    print("Spin mode: ON (slow). Press R/W/A/S/D/Space to exit.")
                else:
                    is_spinning = False
                    stop_all()
                    print("Spin mode: OFF")
                continue
            # === END NEW SPIN MODE ===

            # --- NEW: use current_speed instead of SPEED ---
            if   k == "w":
                # === NEW SPIN MODE: cancel spin on manual command ===
                if is_spinning: is_spinning = False
                # === END NEW SPIN MODE ===
                set_motor(board.M1,  current_speed); set_motor(board.M2, -current_speed)
            elif k == "s":
                # === NEW SPIN MODE: cancel spin on manual command ===
                if is_spinning: is_spinning = False
                # === END NEW SPIN MODE ===
                set_motor(board.M1, -current_speed); set_motor(board.M2,  current_speed)
            elif k == "a":
                # === NEW SPIN MODE: cancel spin on manual command ===
                if is_spinning: is_spinning = False
                # === END NEW SPIN MODE ===
                set_motor(board.M1,               0); set_motor(board.M2, -current_speed)
            elif k == "d":
                # === NEW SPIN MODE: cancel spin on manual command ===
                if is_spinning: is_spinning = False
                # === END NEW SPIN MODE ===
                set_motor(board.M1,  current_speed); set_motor(board.M2,               0)
            elif k == " ":
                # === NEW SPIN MODE: cancel spin on stop ===
                if is_spinning: is_spinning = False
                # === END NEW SPIN MODE ===
                stop_all()
            elif k in {"x", "quit"}:
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop_all()
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)


if __name__ == "__main__":
    main()