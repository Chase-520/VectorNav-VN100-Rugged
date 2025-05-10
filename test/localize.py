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
        self.hp_filter = None
        self.kalman_filters = [Kalman1D(), Kalman1D(), Kalman1D()]

    def update(self, accel, yaw_deg, timestamp):
        if self.prev_time is None:
            self.prev_time = timestamp
            return self.position, self.velocity

        dt = timestamp - self.prev_time
        self.prev_time = timestamp
        if dt <= 0:
            return self.position, self.velocity

        if self.hp_filter is None:
            self.hp_filter = HighPassFilter(cutoff_freq=0.1, dt=dt)

        # High-pass filter
        filtered_accel = self.hp_filter.filter(accel)

        # Convert yaw to radians and rotate acceleration into world frame
        yaw_rad = np.deg2rad(yaw_deg)
        cos_yaw, sin_yaw = np.cos(yaw_rad), np.sin(yaw_rad)

        ax_body, ay_body, az = filtered_accel

        # Rotate acceleration from body frame to world frame (2D rotation)
        ax_world = cos_yaw * ax_body - sin_yaw * ay_body
        ay_world = sin_yaw * ax_body + cos_yaw * ay_body
        az_world = az  # Not rotated

        acc_world = [ax_world, ay_world, az_world]

        for i in range(3):
            self.velocity[i] += acc_world[i] * dt
            self.position[i] += self.velocity[i] * dt
            self.position[i] = self.kalman_filters[i].update(self.position[i], dt)

        return self.position, self.velocity
    
if __name__ == "__main__":
    import time
    from vectornav import Sensor, Registers

    # Initialize and connect to the sensor
    vs = Sensor()
    vs.connect("COM7", Sensor.BaudRate.Baud115200)
    if not vs.verifySensorConnectivity():
        raise Exception("Unable to connect to VectorNav sensor.")

    # reset
    # vs.reset()
    # print("Sensor reset.")
    # time.sleep(2)

    # Set initial settings
    vs.setInitialHeading(0.0)
    # Create an empty register object of the necessary type, where the data member will be populated when the sensor responds to our "read register" request
    modelRegister = Registers.Model()

    vs.readRegister(modelRegister)
    print(f"Sensor Model Number: {modelRegister.model}")

    #
    asyncDataOutputType = Registers.AsyncOutputType()
    asyncDataOutputType.ador = Registers.AsyncOutputType.Ador.YPR
    asyncDataOutputType.serialPort = Registers.AsyncOutputType.SerialPort.Serial1

    vs.writeRegister(asyncDataOutputType)
    print(f"ADOR Configured")
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
    timestamp = 0.0
    try:
        while(timestamp<10):  # Run for 10 seconds
            vs.readRegister(yprMarRegister)
            ax = yprMarRegister.accelX
            ay = yprMarRegister.accelY
            az = yprMarRegister.accelZ
            heading = yprMarRegister.yaw
            timestamp = time.time() - start_time
            position, velocity = imu.update((ax, ay, az), heading, timestamp)


            print(f"t={timestamp:.2f}s | Accel: [{ax:.3f}, {ay:.3f}, {az:.3f}] "
                  f"| Pos: {[round(p, 3) for p in position]} "
                  f"| Vel: {[round(v, 3) for v in velocity]} | Heading: {heading:.2f}°")
            time.sleep(1 / rate)
    except KeyboardInterrupt:
        print("\nStopped by user.")

    vs.disconnect()

