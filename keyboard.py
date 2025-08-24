#!/usr/bin/env python3
import sys, os, tty, termios, signal, time, argparse

from DFRobot_RaspberryPi_DC_Motor import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board

if THIS_BOARD_TYPE:
  board = Board(1, 0x10)    # RaspberryPi select bus 1, set address to 0x10
else:
  board = Board(7, 0x10)    # RockPi select bus 7, set address to 0x10


# --- terminal raw mode helpers ---
class RawTerminal:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
    def __enter__(self):
        tty.setraw(self.fd)
        return self
    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# --- robot control ---
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def set_motor(board, motor, duty_signed, invert=False):
    duty = abs(duty_signed)
    if duty == 0:
        # stop by sending both directions 0 or however your lib stops; here we just send 0 CW
        board.motor_movement([motor], board.CW, 0)
        return
    forward = duty_signed > 0
    if invert: forward = not forward
    direction = board.CW if forward else board.CCW
    board.motor_movement([motor], direction, duty)

def drive_diff(board, left, right, invert_left=False, invert_right=False):
    set_motor(board, board.M1, left, invert_left)
    set_motor(board, board.M2, right, invert_right)

def stop_all(board):
    try:
        board.motor_movement([board.M1], board.CW, 0)
        board.motor_movement([board.M2], board.CW, 0)
    except Exception:
        pass

def main():
    parser = argparse.ArgumentParser(description="Keyboard teleop for DFrobot DC motor shield")
    parser.add_argument("--max", type=int, default=80, help="Max duty (0-100 or board range)")
    parser.add_argument("--min", type=int, default=20, help="Min duty to overcome static friction")
    parser.add_argument("--step", type=int, default=10, help="Duty step for W/S")
    parser.add_argument("--turn", type=int, default=20, help="Extra duty applied as differential on A/D")
    parser.add_argument("--fine", type=int, default=5,  help="Duty step for Q/E")
    parser.add_argument("--invert-left", action="store_true", help="Invert left motor direction")
    parser.add_argument("--invert-right", action="store_true", help="Invert right motor direction")
    parser.add_argument("--rate", type=float, default=30.0, help="Command rate (Hz)")
    args = parser.parse_args()

    duty = 0
    turn = 0
    running = True

    def cleanup_and_quit(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, cleanup_and_quit)
    signal.signal(signal.SIGTERM, cleanup_and_quit)

    help_text = """
Controls:
  W/S : forward/backward (±step)
  A/D : left/right turn (adds ±turn differential)
  Q/E : fine speed down/up (±fine)
  SPACE: EMERGENCY STOP (speed & turn = 0)
  X   : exit
  Arrows work too.
"""
    print(help_text.strip())

    # Non-blocking read one char
    def readch():
        try:
            return os.read(sys.stdin.fileno(), 1)
        except OSError:
            return b""

    # Map escape sequences for arrows
    def read_key():
        c = readch()
        if not c:
            return None
        if c == b"\x03":   # Ctrl-C
            return "QUIT"
        if c == b"\x1b":   # ESC or arrow
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
            b"q":"FINE_DOWN", b"Q":"FINE_DOWN",
            b"e":"FINE_UP",   b"E":"FINE_UP",
            b" ":"STOP",
            b"x":"QUIT", b"X":"QUIT",
            b"h":"HELP", b"H":"HELP",
        }.get(c, None)

    period = 1.0 / args.rate
    last_cmd_time = 0.0

    try:
        with RawTerminal():
            while running:
                t0 = time.time()
                k = read_key()
                if k == "UP":
                    duty = clamp(duty + args.step, 0, args.max)
                elif k == "DOWN":
                    duty = clamp(duty - args.step, 0, args.max)
                elif k == "LEFT":
                    turn = clamp(turn - args.turn, -args.max, args.max)
                elif k == "RIGHT":
                    turn = clamp(turn + args.turn, -args.max, args.max)
                elif k == "FINE_UP":
                    duty = clamp(duty + args.fine, 0, args.max)
                elif k == "FINE_DOWN":
                    duty = clamp(duty - args.fine, 0, args.max)
                elif k == "STOP":
                    duty = 0; turn = 0
                    stop_all(board)
                    print("[STOP]")
                elif k == "HELP":
                    print(help_text.strip())
                elif k == "QUIT":
                    break

                # Compute left/right based on duty & turn
                # Ensure we meet minimum duty when nonzero
                def apply_min(x):
                    if x == 0: return 0
                    sign = 1 if x > 0 else -1
                    return sign * max(args.min, abs(x))

                left_raw  = clamp(duty + (-turn), -args.max, args.max)
                right_raw = clamp(duty + ( turn), -args.max, args.max)
                left_cmd  = apply_min(left_raw) if left_raw != 0 else 0
                right_cmd = apply_min(right_raw) if right_raw != 0 else 0

                # Rate limit actual motor updates
                if time.time() - last_cmd_time >= period:
                    drive_diff(board, left_cmd, right_cmd,
                               invert_left=args.invert_left,
                               invert_right=args.invert_right)
                    last_cmd_time = time.time()

                # Print small status line
                if k is not None:
                    sys.stdout.write(f"\rSpeed:{duty:>3}  Turn:{turn:>4}  L:{left_cmd:>4}  R:{right_cmd:>4}     ")
                    sys.stdout.flush()

                # Sleep to hold loop rate
                dt = time.time() - t0
                if dt < period:
                    time.sleep(period - dt)
    finally:
        stop_all(board)
        print("\nExiting cleanly. Motors stopped.")

if __name__ == "__main__":
    main()