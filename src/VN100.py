from vectornav import Sensor, Registers
import time
import sys

class VN100:
    def __init__(self, port="COM7"):
        self.vs = Sensor()
        self.port = port
        self.baud_rate = Sensor.BaudRate.Baud115200
        self.model_register = Registers.Model()
        self.ypr_mar_register = Registers.YprMagAccelAngularRates()

    def connect(self):
        """ Connect to the VN100 sensor. """
        print("Connecting to VN100...")
        self.vs.connect(self.port, self.baud_rate)
        if not self.vs.verifySensorConnectivity():
            raise Exception(f"Unable to connect to sensor at {self.port} with baud rate {self.baud_rate}")
        print(f"Connected to VN100 at {self.port} with baud rate {self.baud_rate}")

    def set_initial_heading(self, heading=0.0):
        """ Set initial heading of the sensor. """
        self.vs.setInitialHeading(heading)
        print(f"Initial heading set to {heading} degrees.")

    def configure_async_output(self, ador=Registers.AsyncOutputType.Ador.YPR, serial_port=Registers.AsyncOutputType.SerialPort.Serial1):
        """ Configure asynchronous data output type. """
        async_data_output = Registers.AsyncOutputType()
        async_data_output.ador = ador
        async_data_output.serialPort = serial_port
        self.vs.writeRegister(async_data_output)
        print("Asynchronous output configured.")

    def configure_vpe(self, heading_mode=Registers.VpeBasicControl.HeadingMode.Relative, filtering_mode=Registers.VpeBasicControl.FilteringMode.AdaptivelyFiltered):
        """ Configure VPE (Vector Processing Engine). """
        vpe_control = Registers.VpeBasicControl()
        vpe_control.headingMode = heading_mode
        vpe_control.filteringMode = filtering_mode
        self.vs.writeRegister(vpe_control)
        print("VPE Control configured.")

    def read_sensor_data(self):
        """ Read and return sensor data as numeric values. """
        self.vs.readRegister(self.ypr_mar_register)
        imu_data = {
            'Yaw': self.ypr_mar_register.yaw,  # Store as a float
            'Pitch': self.ypr_mar_register.pitch,  # Store as a float
            'Roll': self.ypr_mar_register.roll,  # Store as a float
            'AccelX': self.ypr_mar_register.accelX,  # Store as a float
            'AccelY': self.ypr_mar_register.accelY,  # Store as a float
            'AccelZ': self.ypr_mar_register.accelZ,  # Store as a float
            'MagX': self.ypr_mar_register.magX,  # Store as a float
            'MagY': self.ypr_mar_register.magY,  # Store as a float
            'MagZ': self.ypr_mar_register.magZ,  # Store as a float
            'GyroX': self.ypr_mar_register.gyroX,  # Store as a float
            'GyroY': self.ypr_mar_register.gyroY,  # Store as a float
            'GyroZ': self.ypr_mar_register.gyroZ,  # Store as a float
        }
        return imu_data

    def stream_data(self, duration=10, rate=20):
        """ Stream IMU data for a given duration and rate. """
        print("\nStreaming IMU data (Press Ctrl+C to stop):\n")
        start_time = time.time()
        timestamp = 0.0

        try:
            while timestamp < duration:
                imu_data = self.read_sensor_data()
                output = ' | '.join(f'{k}: {imu_data[k]:.2f}' for k in imu_data)  # Format for display
                sys.stdout.write(f'\r{output.ljust(140)}')
                sys.stdout.flush()
                timestamp = time.time() - start_time
                time.sleep(1 / rate)  # Rate control

        except KeyboardInterrupt:
            print("\n\nStreaming stopped by user.")


    def disconnect(self):
        """ Disconnect from the sensor. """
        self.vs.disconnect()
        print("Disconnected successfully.")

# Usage Example
if __name__ == "__main__":
    sensor = VN100(port="COM7")
    sensor.connect()
    sensor.set_initial_heading(0.0)
    sensor.configure_async_output()
    sensor.configure_vpe()
    sensor.stream_data(duration=10, rate=20)
    sensor.disconnect()
