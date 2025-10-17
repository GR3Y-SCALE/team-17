import io
import fcntl
import time

class GrippyController:
    I2C_SLAVE = 0x0703
    I2C_ADDRESS = 0x08  # Arduino Nano I2C address

    def __init__(self, i2c_bus="/dev/i2c-1"):
        self.bus = open(i2c_bus, "wb", buffering=0)
        fcntl.ioctl(self.bus, self.I2C_SLAVE, self.I2C_ADDRESS)

    def send(self, cmd: str):
        """Send raw ASCII command to the Nano."""
        if not cmd:
            return
        if not isinstance(cmd, (bytes, bytearray)):
            cmd = cmd.encode("ascii")
        self.bus.write(cmd)
        print(f"[I2C] Sent: {cmd.decode()}")

    # -----------------------------
    # LED control
    # -----------------------------
    def led_green(self):  self.send("G")
    def led_yellow(self): self.send("Y")
    def led_red(self):    self.send("R")
    def led_off(self):    self.send("F")

    # -----------------------------
    # Gripper control
    # -----------------------------
    def gripper_open(self):  self.send("O")
    def gripper_close(self): self.send("C")

    # -----------------------------
    # Lift control
    # -----------------------------
    def lift(self, level: int):
        if 0 <= level <= 4:
            self.send(f"L{level}")
        else:
            print(f"[WARN] Invalid lift level: {level}")

    def close(self):
        """Safely close I2C connection."""
        self.bus.close()
        print("[I2C] Connection closed.")


if __name__ == "main":
    # Example test sequence
    grippy = GrippyController()

    print("LED TEST")
    grippy.led_green(); time.sleep(1)
    grippy.led_yellow(); time.sleep(1)
    grippy.led_red(); time.sleep(1)
    grippy.led_off()

    print("GRIPPER TEST")
    grippy.gripper_open(); time.sleep(1)
    grippy.gripper_close()

    print("LIFT TEST")
    for i in range(5):
        grippy.lift(i)
        time.sleep(0.5)

    grippy.led_green()
    grippy.close()
