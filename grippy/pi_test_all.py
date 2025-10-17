import io, fcntl, time

I2C_SLAVE = 0x0703
I2C_ADDRESS = 0x08

with open("/dev/i2c-1", "wb", buffering=0) as f:
    fcntl.ioctl(f, I2C_SLAVE, I2C_ADDRESS)

    # --- LED test ---
    for cmd in [b"G", b"Y", b"R", b"F"]:
        f.write(cmd)
        time.sleep(3)

    # --- Gripper test ---
    f.write(b"O")  # Open
    time.sleep(3)
    f.write(b"C")  # Close
    time.sleep(3)

    # --- Lift test ---
    for cmd in [b"L0", b"L1", b"L2", b"L3", b"L4"]:
        f.write(cmd)
        time.sleep(3)
    