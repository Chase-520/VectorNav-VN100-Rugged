import numpy as np
from scipy.signal import butter, lfilter_zi, lfilter
import time
from src.VN100 import VN100


class HighPassFilter:
    def __init__(self, cutoff_freq, fs, order=4):
        self.cutoff = cutoff_freq
        self.fs = fs
        self.order = order

        # Design filter coefficients
        self.b, self.a = butter(order, cutoff_freq / (0.5 * fs), btype='high', analog=False)

        # Initialize filter states for 3 axes (x, y, z)
        self.zi = [lfilter_zi(self.b, self.a) * 0.0 for _ in range(3)]  # start with zero input
        self.last_output = [0.0, 0.0, 0.0]

    def filter(self, accel):
        filtered = []
        for i in range(3):
            y, self.zi[i] = lfilter(self.b, self.a, [accel[i]], zi=self.zi[i])
            filtered.append(y[0])
        return filtered


class Kalman1D:
    def __init__(self):
        self.x = 0.0     # State (e.g., position)
        self.v = 0.0     # Velocity
        self.p = 1.0     # Estimation error covariance
        self.q = 0.01    # Process noise
        self.r = 0.1     # Measurement noise

    def update(self, measurement, dt):
        # Predict
        self.x += self.v * dt
        self.p += self.q

        # Update
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1 - k)

        return self.x


class Localization:
    def __init__(self):
        self.prev_time = None
        self.velocity = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0, 0.0]
        self.hp_filter = None
        self.kalman_filters = [Kalman1D(), Kalman1D(), Kalman1D()]

    def update(self, accel, yaw_deg, timestamp):
        """
        Update the position and velocity of the system using accelerometer data and yaw.
        
        accel: Tuple of (ax, ay, az) accelerometer data in body frame.
        yaw_deg: Current heading of the sensor in degrees (used to rotate accelerometer data).
        timestamp: Current time in seconds.
        
        Returns the updated position and velocity.
        """
        if self.prev_time is None:
            self.prev_time = timestamp
            return self.position, self.velocity

        dt = timestamp - self.prev_time
        self.prev_time = timestamp
        if dt <= 0:
            return self.position, self.velocity

        if self.hp_filter is None:
            self.hp_filter = HighPassFilter(cutoff_freq=0.1, fs=1.0 / dt)

        # High-pass filter to remove low-frequency components (gravity)
        filtered_accel = self.hp_filter.filter(accel)

        # Convert yaw to radians and rotate acceleration into world frame
        yaw_rad = np.deg2rad(yaw_deg)
        cos_yaw, sin_yaw = np.cos(yaw_rad), np.sin(yaw_rad)

        ax_body, ay_body, az = filtered_accel

        # Rotate acceleration from body frame to world frame (2D rotation)
        ax_world = cos_yaw * ax_body - sin_yaw * ay_body
        ay_world = sin_yaw * ax_body + cos_yaw * ay_body
        az_world = az  # Not rotated for simplicity in this example

        acc_world = [ax_world, ay_world, az_world]

        # Update velocity and position
        for i in range(3):
            self.velocity[i] += acc_world[i] * dt
            self.position[i] += self.velocity[i] * dt
            # Use Kalman filter to smooth position estimates
            self.position[i] = self.kalman_filters[i].update(self.position[i], dt)

        return self.position, self.velocity


if __name__ == "__main__":
    import sys
    # Sample data for testing
    accel_data = [0.05, -0.01, -9.81]  # Accelerometer readings (ax, ay, az)
    yaw_angle = 30.0  # Heading in degrees
    timestamp = 0.1  # Initial timestamp

    # Initialize the localization class
    localization = Localization()

    # Initialize the VN100 sensor
    vs = VN100(port="COM7")
    vs.connect()
    vs.set_initial_heading(0.0)
    vs.configure_async_output()
    vs.configure_vpe()

    # Simulate updates
    try:
        start_time = time.time()
        while True:
            # Read data from the sensor
            data = vs.read_sensor_data()    

            # Ensure data contains the necessary accelerometer and yaw information
            if "AccelX" not in data or "AccelY" not in data or "AccelZ" not in data or "Yaw" not in data:
                print("Error: Missing sensor data.")
                break

            # Extract accelerometer and yaw data
            accel_data = [data["AccelX"], data["AccelY"], data["AccelZ"]]
            yaw_angle = data["Yaw"]

            # Calculate the timestamp
            timestamp = time.time() - start_time

            # Update localization with accelerometer data and yaw
            position, velocity = localization.update(accel_data, yaw_angle, timestamp)

            # Print out the current position and velocity
            # Print out the current position and velocity
            output = f"Time: {timestamp:.2f}s | Position: {[round(float(x), 4) for x in position]} | Velocity: {[round(float(x), 4) for x in velocity]}"
            sys.stdout.write(f'\r{output.ljust(140)}')  # Overwrites the previous line in the terminal
            sys.stdout.flush()


            time.sleep(0.05)  # Small delay to avoid spamming output too fast

    except KeyboardInterrupt:
        print("\n\nStreaming stopped by user.")

    # Disconnect the sensor
    vs.disconnect()
    print("Disconnected successfully.")
