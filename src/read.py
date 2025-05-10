import serial
import time
import sys
import csv
import os
from datetime import datetime

# Configuration
DEFAULT_PORT = "COM7"
BAUD_RATE = 115200
FREQ_HZ = 80
CSV_FILENAME = f"imu_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# IMU data keys
FIELDS = ["Yaw", "Pitch", "Roll", "MagX", "MagY", "MagZ",
          "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ"]

def read_imu_line(line):
    """
    Parses an IMU data line and returns a dictionary.
    """
    if not line.startswith("$VNYMR"):
        return None

    parts = line.strip().split(",")
    if len(parts) < 13:
        return None

    try:
        return {
            key: parts[i + 1].split("*")[0] if key == "GyroZ" else parts[i + 1]
            for i, key in enumerate(FIELDS)
        }
    except IndexError:
        return None

def main(serial_port=DEFAULT_PORT, csv_file=CSV_FILENAME):
    try:
        with serial.Serial(serial_port, baudrate=BAUD_RATE, timeout=1) as ser, \
             open(csv_file, 'w', newline='') as f:

            print(f"Connected to {ser.name}, saving to {csv_file}")
            writer = csv.writer(f)
            writer.writerow(["time_ns"] + FIELDS)

            print("Reading IMU data at 80 Hz. Press Ctrl+C to stop.")

            while True:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore')
                    imu_data = read_imu_line(line)

                    if imu_data:
                        timestamp = time.time_ns()
                        row = [timestamp] + [imu_data[key] for key in FIELDS]
                        writer.writerow(row)

                        # Print data to console
                        output = ' | '.join(f'{k}: {imu_data[k]}' for k in FIELDS)
                        sys.stdout.write(f'\r{output.ljust(140)}')
                        sys.stdout.flush()

                    time.sleep(1 / FREQ_HZ)

                except Exception as e:
                    print(f"\nError reading/parsing IMU data: {e}")

    except serial.SerialException as e:
        print(f"Serial port error: {e}")
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        print("Exiting program.")

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    main(serial_port=port)
