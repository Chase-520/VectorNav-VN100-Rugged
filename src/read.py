import serial
import time
import sys

# Open serial port (adjust as needed for your platform)
ser = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=1)
print(f"Connected to {ser.name}")

elements = ["Yaw", "Pitch", "Roll", "MagX", "MagY", "MagZ",
            "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ"]

def read_imu(ser):
    """
    Reads and parses a line of IMU data from serial.
    Returns a dictionary with keys for each sensor measurement.
    """
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line.startswith("$VNYMR"):
        x = line.split(",")
        if len(x) >= 13:
            data = {}
            for i, key in enumerate(elements):
                value = x[i+1]
                if key == "GyroZ":
                    value = value.split("*")[0]  # remove checksum
                data[key] = value
            return data
    return None

try:
    print("Reading IMU data at 10 Hz. Press Ctrl+C to stop.")
    while True:
        imu_data = read_imu(ser)
        if imu_data:
            output = ' | '.join(f'{k}: {v}' for k, v in imu_data.items())
            sys.stdout.write(f'\r{output.ljust(140)}')
            sys.stdout.flush()
        time.sleep(1/80)

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    ser.close()
    print("Serial port closed.")
