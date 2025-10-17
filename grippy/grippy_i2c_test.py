import fcntl
import time

class GrippyController:
    I2C_SLAVE = 0x0703
    I2C_ADDRESS = 0x08  # Arduino Nano I2C address

    def __init__(self, i2c_bus="/dev/i2c-1", verbose=True):
        self.bus = open(i2c_bus, "wb", buffering=0)
        fcntl.ioctl(self.bus, self.I2C_SLAVE, self.I2C_ADDRESS)
        self.verbose = verbose
        if self.verbose:
            print("[I2C] Connected to Arduino at 0x08")

    def send(self, cmd: str):
        """Send raw ASCII command string to Arduino."""
        if not cmd:
            return
        if not isinstance(cmd, (bytes, bytearray)):
            cmd = cmd.encode("ascii")
        self.bus.write(cmd)
        if self.verbose:
            print(f"[I2C] Sent: {cmd.decode()}")

    # LED
    def led_green(self):  self.send("G")
    def led_yellow(self): self.send("Y")
    def led_red(self):    self.send("R")
    def led_off(self):    self.send("F")

    # Gripper
    def gripper_open(self):  self.send("O")
    def gripper_close(self): self.send("C")

    # Lift
    def lift(self, angle: int):
        if 0 <= angle <= 180:
            self.send(f"L{angle}")
        else:
            print("[WARN] Lift angle must be 0-180")

    def close(self):
        self.bus.close()
        if self.verbose:
            print("[I2C] Connection closed")


if __name__ == "__main__":
    grippy = GrippyController(verbose=True)
    print("=== Grippy I2C Interactive Test ===")
    print("Commands: O=Open, C=Close, L<number>=Lift angle, G/Y/R/F=LEDs, Q=Quit")

    try:
        while True:
            cmd = input("Enter command: ").strip()
            if cmd.upper() == "Q":
                break
            elif cmd.upper() == "O":
                grippy.gripper_open()
            elif cmd.upper() == "C":
                grippy.gripper_close()
            elif cmd.upper().startswith("L"):
                try:
                    angle = int(cmd[1:])
                    grippy.lift(angle)
                except ValueError:
                    print("[ERROR] Invalid lift angle")
            elif cmd.upper() == "G":
                grippy.led_green()
            elif cmd.upper() == "Y":
                grippy.led_yellow()
            elif cmd.upper() == "R":
                grippy.led_red()
            elif cmd.upper() == "F":
                grippy.led_off()
            else:
                print("[ERROR] Unknown command")
    finally:
        grippy.close()

