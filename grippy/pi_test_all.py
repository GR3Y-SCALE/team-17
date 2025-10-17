import io, fcntl, time

I2C_SLAVE = 0x0703
I2C_ADDRESS = 0x08

with open("/dev/i2c-1", "wb", buffering=0) as f:
    fcntl.ioctl(f, I2C_SLAVE, I2C_ADDRESS)

   

    # --- Gripper test ---
    f.write(b"O")  # Open
    time.sleep(3)
    f.write(b"C")  # Close
    time.sleep(3)

    
    