import io
import fcntl
import time

I2C_SLAVE = 0x0703
I2C_ADDRESS = 0x08

class GrippyController:
    def __init__(self, i2c_bus="/dev/i2c-1"):
        try:
            self.bus = open(i2c_bus, "wb", buffering=0)
            fcntl.ioctl(self.bus, I2C_SLAVE, I2C_ADDRESS)
            print("[ OK ] Initalised item collection.")
        except:
            print("[ ERROR ] Could not communicate with item collection")

    def send(self, cmd: str):
        """Send a command string to the Arduino via I2C."""
        self.bus.write(cmd.encode())

    # --- LED Control ---
    def led_green(self): self.send("G")
    def led_yellow(self): self.send("Y")
    def led_red(self): self.send("R")
    def led_off(self): self.send("F")

    # --- Gripper Control ---
    def gripper_open(self): self.send("O")
    def gripper_close(self): self.send("C")

    # --- Lift Control ---
    def lift(self, angle: int):
        if 0 <= angle <= 180:
            self.send(f"L{angle}")
        else:
            print("[ ERROR ] Lift angle is between 0-180.")

    def cleanup(self):
        self.bus.close()

# === Example usage ===
if __name__ == "__main__":
    g = GrippyController()

    # Example test sequence (remove when integrating)
    g.led_green()
    time.sleep(0.5)
    g.gripper_close()
    time.sleep(0.5)
    g.lift(3)
    time.sleep(0.5)
    g.led_red()
    g.cleanup()
