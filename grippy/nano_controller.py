import io, fcntl, time

I2C_SLAVE = 0x0703 # control code for linux for IO I2C
I2C_ADDRESS = 0x08

with open("/dev/i2c-1", "wb", buffering=0) as f:
    fcntl.ioctl(f, I2C_SLAVE, I2C_ADDRESS)
    f.write(b"G")
    time.sleep(1)
    f.write(b"R")
    time.sleep(1)
    f.write(b"F")
