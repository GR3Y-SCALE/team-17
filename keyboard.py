#!/usr/bin/env python3
import sys, os, tty, termios, signal, time

from DFRobot_RaspberryPi_DC_Motor import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board

# ----- Board setup -----
board = Board(1 if THIS_BOARD_TYPE else 7, 0x10)
while board.begin() != board.STA_OK:
    pass
print('board initialised')

# Choose motor directions for "forward". If your robot drives backward, swap these.
FORWARD = board.CW
REVERSE = board.CCW

# ----- Raw terminal helper -----
class RawTerminal:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
    def __enter__(self):
        tty.setraw(self.fd)
        return self
    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ----- Motor helpers -----
def set_motor(motor, value):
    """value in [-100..100]; sign = direction, abs = duty"""
    value = max(-100, min(100, int(value)))
    if value == 0:
        board.motor_movement([motor], FORWARD, 0)
        return
    dir_ = FORWARD if value > 0 else REVERSE
    board.motor_movement([motor], dir_, abs(value))

def stop_all():
    try:
        board.motor_movement([board.M1], FORWARD, 0)
        board.motor_movement([board.M2], FORWARD, 0)
    except Exception:
        pass

# ----- Key reading -----
def readch():
    try:
        return os.read(sys.stdin.fileno(), 1)
    except OSError:
        return b""

def read_key():
    c = readch()
    if not c:
        return None
    if c == b"\x03":     # Ctrl-C
        return "QUIT"
    if c == b"\x1b":     # Arrow keys
        c2 = readch()
        if c2 == b"[":
            c3 = readch()
            return {b"A":"UP", b"B":"DOWN", b"C":"RIGHT", b"D":"LEFT"}.get(c3, None)
        return None
    return {
        b"w":"UP", b"W":"UP",
        b"s":"DOWN", b"S":"DOWN",
        b"a":"LEFT", b"A":"LEFT",
        b"d":"RIGHT", b"D":"RIGHT",
        b" ":"STOP",
        b"x":"QUIT", b"X":"QUIT",
    }.get(c, None)

# ----- Main loop -----
def main():
    running = True
    def cleanup_and_quit(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, cleanup_and_quit)
    signal.signal(signal.SIGTERM, cleanup_and_quit)

    duty = 100
    cur_M1 = cur_M2 = None  # force initial write

    try:
        with RawTerminal():
            while running:
                # default to no movement; a key press will override this tick
                M1 = 0
                M2 = 0

                k = read_key()
                if k == "UP":
                    M1 = duty;  M2 = duty
                elif k == "DOWN":
                    M1 = -duty; M2 = -duty
                elif k == "LEFT":
                    # gentle left: right wheel forward, left wheel stop
                    M1 = 0;     M2 = duty
                elif k == "RIGHT":
                    # gentle right: left wheel forward, right wheel stop
                    M1 = duty;  M2 = 0
                elif k == "STOP":
                    M1 = 0; M2 = 0
                elif k == "QUIT":
                    break
                # else: no key -> keep stopped this tick

                # Only send if changed
                if (M1, M2) != (cur_M1, cur_M2):
                    set_motor(board.M1, M1)
                    set_motor(board.M2, M2)
                    cur_M1, cur_M2 = M1, M2

                time.sleep(0.02)  # ~50 Hz loop
    finally:
        stop_all()
        board.motor_stop(board.ALL)

if __name__ == "__main__":
    main()