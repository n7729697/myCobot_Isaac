import serial
import re

# Serial port configuration
port = 'COM13'
baudrate = 115200  # adjust to your setup

# Regular expression to extract p1 value
pattern = re.compile(r"p1:(\d+\.\d+)")

try:
    with serial.Serial(port, baudrate, timeout=1) as ser:
        print(f"Listening on {port}...")
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue

            match = pattern.search(line)
            if match:
                p1_value = float(match.group(1))
                print(f"p1 = {p1_value}")
except KeyboardInterrupt:
    print("\nStopped by user.")
except serial.SerialException as e:
    print(f"Serial error: {e}")
