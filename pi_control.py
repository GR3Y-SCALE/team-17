import serial
import time
import sys
import glob

def find_pico_port():
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    return ports[0] if ports else None

def main():
    port = find_pico_port()
    if not port:
        print("No Pico serial port found.")
        sys.exit(1)

    print(f"Connecting to Pico on {port}...")
    pico = serial.Serial(port, 115200, timeout=2)
    time.sleep(2)
    print("Connected.\n")

    commands = ["L0", "G0", "L3", "G1"]
    print("Example sequence starting...\n")

    for cmd in commands:
        line = f"{cmd}\n"
        pico.write(line.encode())
        print(f"Sent: {cmd}")
        reply = pico.readline().decode(errors="ignore").strip()
        if reply:
            print(f"PICO: {reply}")
        time.sleep(1)

    pico.close()
    print("\nSequence done. Port closed.")

if __name__ == "__main__":
    main()
