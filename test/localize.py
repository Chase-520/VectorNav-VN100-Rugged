import numpy as np
from scipy.signal import butter, lfilter_zi, lfilter
import time
from src.VN100 import VN100


class HighPassFilter:
    def __init__(self, alpha=0.9):
        self.alpha = alpha
        self.prev_input = [0.0, 0.0, 0.0]
        self.prev_output = [0.0, 0.0, 0.0]

    def filter(self, accel):
        filtered = []
        for i in range(3):
            y = self.alpha * (self.prev_output[i] + accel[i] - self.prev_input[i])
            filtered.append(y)
            self.prev_output[i] = y
            self.prev_input[i] = accel[i]
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
            self.hp_filter = HighPassFilter(alpha=0.7)

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

        # Update velocity and position (no Kalman filtering)
        for i in range(3):
            self.velocity[i] += acc_world[i] * dt
            self.position[i] += self.velocity[i] * dt



        return self.position, self.velocity, filtered_accel
    
    def visualize_position_velocity_accel(self,sensor, duration=20, rate=80):
        """Real-time plot of position, velocity, and acceleration in one window with separate rows."""
        from pyqtgraph.Qt import QtCore, QtWidgets
        import pyqtgraph as pg
        import time

        # Create Qt application
        app = QtWidgets.QApplication([])

        # Create a single main window
        win = pg.GraphicsLayoutWidget(title="VN100 Position, Velocity, Acceleration")
        win.resize(1000, 900)

        # Plot titles and colors
        colors = ['r', 'g', 'b']
        labels = ['X', 'Y', 'Z']

        # Position Plot
        pos_plot = win.addPlot(title="Position (X, Y, Z)")
        pos_plot.addLegend()
        pos_curves = [pos_plot.plot(pen=c, name=f'{l} Position') for c, l in zip(colors, labels)]
        win.nextRow()

        # Velocity Plot
        vel_plot = win.addPlot(title="Velocity (Vx, Vy, Vz)")
        vel_plot.addLegend()
        vel_curves = [vel_plot.plot(pen=c, name=f'{l} Velocity') for c, l in zip(colors, labels)]
        win.nextRow()

        # Acceleration Plot
        accel_plot = win.addPlot(title="Acceleration (Ax, Ay, Az)")
        accel_plot.addLegend()
        accel_curves = [accel_plot.plot(pen=c, name=f'{l} Accel') for c, l in zip(colors, labels)]

        # Initialize data buffers
        data_len = int(duration * rate)
        x_vals = list(range(-data_len, 0))
        buffers = {
            'position': [[0.0] * data_len for _ in range(3)],
            'velocity': [[0.0] * data_len for _ in range(3)],
            'accel':    [[0.0] * data_len for _ in range(3)],
        }

        # Show window
        win.show()
        self._start_time = time.time()

        # Timer callback
        def update():
            nonlocal buffers

            # Read sensor data
            data = sensor.read_sensor_data()
            accel = [data["AccelX"], data["AccelY"], data["AccelZ"]]
            yaw = data["Yaw"]
            now = time.time() - self._start_time

            # Update localization (requires self.update method)
            pos, vel, filtered_accel = self.update(accel, 0, now)

            # Update buffers
            for i in range(3):
                buffers['position'][i] = buffers['position'][i][1:] + [pos[i]]
                buffers['velocity'][i] = buffers['velocity'][i][1:] + [vel[i]]
                buffers['accel'][i]    = buffers['accel'][i][1:]    + [filtered_accel[i]]

            # Plot data
            for i in range(3):
                pos_curves[i].setData(x_vals, buffers['position'][i])
                vel_curves[i].setData(x_vals, buffers['velocity'][i])
                accel_curves[i].setData(x_vals, buffers['accel'][i])

        # Timer setup
        timer = QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(int(1000 / rate))

        # Start Qt event loop
        app.exec_()


if __name__ == "__main__":
    localization = Localization()

    vs = VN100(port="COM7")
    vs.connect()
    vs.configure_initial_heading(0.0)
    vs.configure_async_output()
    vs.configure_vpe()
    vs.configure_filter()
    vs.configure_bias()

    localization.vs = vs  # Inject VN100 into Localization

    try:
        localization.visualize_position_velocity_accel(duration=20, rate=80, sensor=vs)
    except KeyboardInterrupt:
        print("\nVisualization stopped.")
    finally:
        vs.disconnect()
        print("Disconnected successfully.")
