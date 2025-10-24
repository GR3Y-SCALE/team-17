# keyboardControl_tui.py
# Terminal-only (SSH-safe) keyboard control using curses (no X11).
# Press arrows or WASD to drive; ESC or 'q' to quit.


import curses
import time,os,sys,signal
# sys.path.insert(1, '../mobility')
# from mobility.drive_system import DriveSystem
from grippy.grippy_i2c import GrippyController
from mobility.drive_system import DriveSystem


IDLE_STOP_MS = 120   # stop if no keypress seen for this many ms
FWD = 0.9            # adjust to your units (e.g., m/s or duty)
TURN = 300.0          # adjust to your units

robot = DriveSystem()
gripper = GrippyController()
_last_input_time = time.monotonic()

position = [135,95,50,0]

def stop_robot():
    try:
        robot.stop_all()
    except Exception:
        pass

def _handle_exit(signum, frame):
    stop_robot()
    raise SystemExit

signal.signal(signal.SIGINT, _handle_exit)
signal.signal(signal.SIGTERM, _handle_exit)

def drive_up():
    robot.set_target_velocities(FWD, 0.0)

def drive_down():
    robot.set_target_velocities(-FWD, 0.0)

def turn_left():
    robot.set_target_velocities(0.0, -TURN)

def turn_right():
    robot.set_target_velocities(0.0, TURN)

def main(stdscr):
    global _last_input_time
    curses.cbreak()
    curses.noecho()
    stdscr.keypad(True)
    stdscr.nodelay(True)  # non-blocking getch
    stdscr.clear()
    stdscr.addstr(0, 0, "Arrow keys/WASD to drive. ESC or 'q' to quit.")
    stdscr.addstr(1, 0, f"Idle-stop: {IDLE_STOP_MS} ms")
    stdscr.refresh()

    try:
        while True:
            ch = stdscr.getch()
            now = time.monotonic()

            if ch != -1:
                _last_input_time = now

                if ch in (curses.KEY_UP, ord('w'), ord('W')):
                    drive_up()
                elif ch in (curses.KEY_DOWN, ord('s'), ord('S')):
                    drive_down()
                elif ch in (curses.KEY_LEFT, ord('a'), ord('A')):
                    turn_left()
                elif ch in (curses.KEY_RIGHT, ord('d'), ord('D')):
                    turn_right()
                elif ch == ord('1'):
                    gripper.lift(position[0])
                elif ch == ord('2'):
                    gripper.lift(position[1])
                elif ch == ord('3'):
                    gripper.lift(position[2])
                elif ch == ord('4'):
                    gripper.lift(position[3])
                elif ch in (ord('q'), ord('Q')):
                    gripper.gripper_open()
                elif ch in (ord('e'), ord('E')):
                    gripper.gripper_close()
                elif ch == 27:  # ESC
                    break
                

            # terminals don't send key-up; emulate "release to stop"
            if (now - _last_input_time) * 1000.0 > IDLE_STOP_MS:
                stop_robot()
                # push the timer forward so we don't spam stop_all()
                _last_input_time = now

            time.sleep(0.01)  # ~100 Hz loop

    finally:
        stop_robot()
        gripper.gripper_open()
        # restore terminal settings
        stdscr.nodelay(False)
        stdscr.keypad(False)
        curses.echo()

if __name__ == "__main__":
    curses.wrapper(main)