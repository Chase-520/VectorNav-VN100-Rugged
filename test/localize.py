import numpy as np

class HighPassFilter:
    def __init__(self, cutoff_freq, dt):
        self.alpha = 1 / (1 + (1 / (2 * np.pi * cutoff_freq * dt)))
        self.prev_input = [0.0, 0.0, 0.0]
        self.prev_output = [0.0, 0.0, 0.0]

    def filter(self, accel):
        output = []
        for i in range(3):
            y = self.alpha * (self.prev_output[i] + accel[i] - self.prev_input[i])
            output.append(y)
            self.prev_input[i] = accel[i]
            self.prev_output[i] = y
        return output


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


class ImuDeadReckoning:
    def __init__(self):
        self.prev_time = None
        self.velocity = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0, 0.0]

        # Filters for each axis
        self.hp_filter = None  # Initialized per timestep
        self.kalman_filters = [Kalman1D(), Kalman1D(), Kalman1D()]

    def update(self, accel, timestamp):
        if self.prev_time is None:
            self.prev_time = timestamp
            return self.position, self.velocity

        dt = timestamp - self.prev_time
        self.prev_time = timestamp
        if dt <= 0:
            return self.position, self.velocity

        if self.hp_filter is None:
            self.hp_filter = HighPassFilter(cutoff_freq=0.1, dt=dt)

        filtered_accel = self.hp_filter.filter(accel)

        for i in range(3):
            self.velocity[i] += filtered_accel[i] * dt
            self.position[i] += self.velocity[i] * dt
            self.position[i] = self.kalman_filters[i].update(self.position[i], dt)

        return self.position, self.velocity


if __name__ == "__main__":
    import time
    from vectornav import Sensor, Registers

    # Initialize and connect to the sensor
    vs = Sensor()
    vs.connect("/dev/ttyUSB0", Sensor.BaudRate.Baud115200)
    if not vs.verifySensorConnectivity():
        raise Exception("Unable to connect to VectorNav sensor.")

    # Set initial settings
    vs.setInitialHeading(0.0)
    vpeControl = Registers.VpeBasicControl()
    vpeControl.headingMode = Registers.VpeBasicControl.HeadingMode.Absolute
    vpeControl.filteringMode = Registers.VpeBasicControl.FilteringMode.AdaptivelyFiltered
    vpeControl.tuningMode = Registers.VpeBasicControl.TuningMode.Adaptive
    vs.writeRegister(vpeControl)

    # Register to read acceleration data
    yprMarRegister = Registers.YprMagAccelAngularRates()

    imu = ImuDeadReckoning()
    start_time = time.time()

    rate = 20  # Hz
    try:
        for _ in range(rate * 10):  # Run for 10 seconds
            vs.readRegister(yprMarRegister)
            ax = yprMarRegister.accelX
            ay = yprMarRegister.accelY
            az = yprMarRegister.accelZ

            timestamp = time.time() - start_time
            position, velocity = imu.update((ax, ay, az), timestamp)

            print(f"t={timestamp:.2f}s | Accel: [{ax:.3f}, {ay:.3f}, {az:.3f}] "
                  f"| Pos: {[round(p, 3) for p in position]} "
                  f"| Vel: {[round(v, 3) for v in velocity]}")
            time.sleep(1 / rate)
    except KeyboardInterrupt:
        print("\nStopped by user.")

