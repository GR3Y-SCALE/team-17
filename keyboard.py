#!/usr/bin/env python3
import sys, os, tty, termios, signal, time
from DFRobot_RaspberryPi_DC_Motor import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board

if THIS_BOARD_TYPE:
    board = Board(1, 0x10)
else:
    board = Board(7, 0x10)

mot_state = [board.CCW, board.CW]

class RawTerminal:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
    def __enter__(self):
        tty.setraw(self.fd)
        return self
    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

def set_motor(board, motor_name, signed_duty, direction):
    if signed_duty < 0:
        direction = ~direction
    board.motor_movement([getattr(board, motor_name)], mot_state[direction], int(abs(signed_duty)))

def stop_all(board):
    try:
        board.motor_movement([board.M1], board.CW, 0)
        board.motor_movement([board.M2], board.CW, 0)
    except Exception:
        pass

def main():
    duty = 0
    turn = 0
    running = True

    def cleanup_and_quit(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, cleanup_and_quit)
    signal.signal(signal.SIGTERM, cleanup_and_quit)

    def readch():
        try:
            return os.read(sys.stdin.fileno(), 1)
        except OSError:
            return b""

    def read_key():
        c = readch()
        if not c:
            return None
        if c == b"\x03":
            return "QUIT"
        if c == b"\x1b":
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
        }.get(c, None)

    period = 0.1
    last_cmd_time = 0.0

    try:
        with RawTerminal():
            while running:
                t0 = time.time()
                k = read_key()
                if k == "UP":
                    duty += 10
                elif k == "DOWN":
                    duty -= 10
                elif k == "LEFT":
                    turn += 10
                elif k == "RIGHT":
                    turn -= 10
                elif k == "STOP":
                    duty = 0; turn = 0
                    stop_all(board)
                elif k == "QUIT":
                    break

                if time.time() - last_cmd_time >= period:
                    set_motor(
                        board, 
                        'M1', 
                        duty - duty*(turn/100), # Diferential
                        0 if duty < 0 else 1 # motor direction CCW or CW
                    )
                    set_motor(
                        board, 
                        'M2', 
                        duty + duty*(turn/100), 
                        1 if duty < 0 else 0
                    )
                    last_cmd_time = time.time()

                dt = time.time() - t0
                if dt < period:
                    time.sleep(period - dt)
    finally:
        board.motor_stop(board.ALL)

if __name__ == "__main__":
    main()