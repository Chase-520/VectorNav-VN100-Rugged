import vectornav
from vectornav import Sensor, ByteBuffer, Registers
from vectornav.Plugins import FileExporter, SimpleLogger, ExporterCsv
import os
import time
from datetime import datetime
import sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import traceback

def export_bin_to_csv(bin_file_path: str, output_folder: str, with_timestamps: bool = False):
    """
    Converts a .bin file recorded with SimpleLogger into multiple CSV files using VectorNav SDK.
    
    Parameters:
    - bin_file_path (str): Path to the .bin file.
    - output_folder (str): Folder where CSV files will be saved.
    - with_timestamps (bool): If True, include system timestamps in CSV.
    """
    if not os.path.isfile(bin_file_path):
        raise FileNotFoundError(f"Binary file not found: {bin_file_path}")

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    file_exporter = FileExporter()
    file_exporter.addCsvExporter(output_folder, enableSystemTimeStamps=with_timestamps)
    
    print(f"Exporting {bin_file_path} → CSV files in {output_folder} ...")
    file_exporter.processFile(bin_file_path)
    print("Export complete.")

def parse_baudRate(rate: int):
    if rate==115200:
        rate = Sensor.BaudRate.Baud115200
    elif rate==128000:
        rate = Sensor.BaudRate.Baud128000
    elif rate==230400:
        rate = Sensor.BaudRate.Baud230400
    elif rate==460800:
        rate = Sensor.BaudRate.Baud460800
    else:
        rate = Sensor.BaudRate.Baud115200
    return rate

class VN100:
    def __init__(self, port="COM7", baudrate:int=230400):
        self.vs = Sensor()
        self.port = port
        self.baud_rate = parse_baudRate(baudrate)
        self.model_register = Registers.Model()
        self.ypr_mar_register = Registers.YprMagAccelAngularRates()
        self.CSV_FILENAME = f"imu_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.bias = {
            'GyroX': 0.0, 'GyroY': 0.0, 'GyroZ': 0.0,
            'AccelX': 0.0, 'AccelY': 0.0, 'AccelZ': 0.0
        }

    def connect(self):
        """ Connect to the VN100 sensor. """
        print("Connecting to VN100...")
        self.vs.connect(self.port, self.baud_rate)
        if not self.vs.verifySensorConnectivity():
            self.vs.autoConnect(self.port)
            if not self.vs.verifySensorConnectivity():
                raise Exception(f"Unable to connect to sensor at {self.port} with baud rate {self.baud_rate}")
            print(f"buad rate mismatch, current baud rate {self.vs.connectedBaudRate()}")
        else:
            print(f"Connected to VN100 at {self.port} with baud rate {self.baud_rate}")

        modelRegister = Registers.Model()
        self.vs.readRegister(modelRegister)
        print(f"Sensor Model Number: {modelRegister.model}")

    def configure_baud_rate(self, rate:int = 115200):
        """ Configure the baud rate of the sensor. """
        baudrateRegister = Registers.BaudRate()
        rate = parse_baudRate(rate)
        
        port = Registers.BaudRate.SerialPort.Serial1
        baudrateRegister.baudRate = rate
        baudrateRegister.serialPort = port
        self.vs.writeRegister(baudrateRegister)
        print(f"Baud rate configured to {rate} on {port}")
        
    def calibrate_bias(self, duration=3.0, rate=100):
        """ Calibrate sensor biases over a short period. """
        print(f"Calibrating for {duration} seconds at {rate} Hz... Please keep the sensor still.")
        num_samples = int(duration * rate)

        # Buffers for accumulating values
        bias_accumulators = {
            'GyroX': 0.0, 'GyroY': 0.0, 'GyroZ': 0.0,
            'AccelX': 0.0, 'AccelY': 0.0, 'AccelZ': 0.0
        }

        for _ in range(num_samples):
            data = self.read_sensor_data()
            for key in bias_accumulators:
                bias_accumulators[key] += data[key]
            time.sleep(1.0 / rate)

        # Compute averages
        self.biases = {key: bias_accumulators[key] / num_samples for key in bias_accumulators}
        print("Calibration complete. Estimated biases:")
        for key, val in self.biases.items():
            print(f"{key} bias: {val:.4f}")

    def configure_bias(self):
        import json
        with open("src\sensor_bias.json", "r") as f:
            bias_data = json.load(f)

        gyro_bias = bias_data["gyro_bias"]
        accel_bias = bias_data["accel_bias"]
        self.biases ={
            'GyroX': gyro_bias['x'], 'GyroY': gyro_bias['y'], 'GyroZ': gyro_bias['z'],
            'AccelX': accel_bias['x'], 'AccelY': accel_bias['y'], 'AccelZ': accel_bias['z']
        }
        for key, val in self.biases.items():
            print(f"{key} bias: {val:.4f}")

    def configure_initial_heading(self, heading=0.0):
        """ Set initial heading of the sensor. """
        self.vs.setInitialHeading(heading)
        print(f"Initial heading set to {heading} degrees.")


    def configure_async_output(self, outputType:str = "YPR"):
        """ Configure asynchronous data output type. """
        async_data_output = Registers.AsyncOutputType()
        async_data_output.ador = Registers.AsyncOutputType.Ador.ACC
        async_data_output.serialPort = Registers.AsyncOutputType.SerialPort.Serial1
        self.vs.writeRegister(async_data_output)
        print("Asynchronous output configured.")
    
    def configure_async_output_freq(self):
        asyncDataOutputFreq= Registers.AsyncOutputFreq()
        asyncDataOutputFreq.adof = Registers.AsyncOutputFreq.Adof.Rate100Hz
        asyncDataOutputFreq.serialPort = Registers.AsyncOutputFreq.SerialPort.Serial1
        self.vs.writeRegister(asyncDataOutputFreq)
        print("Asynchronous output frequency configured.")

    def configure_vpe(self):
        """ Configure VPE (Vector Processing Engine). """
        vpe_control = Registers.VpeBasicControl()
        vpe_control.headingMode = Registers.VpeBasicControl.HeadingMode.Relative
        vpe_control.filteringMode = Registers.VpeBasicControl.FilteringMode.AdaptivelyFiltered
        self.vs.writeRegister(vpe_control)
        print("VPE Control configured.")

    def configure_filter(self):
        """configure internal low pass filter"""
        filter = Registers.ImuFilterControl()
        accel_mode= Registers.ImuFilterControl.AccelFilterMode(1)
        gyro_mode = Registers.ImuFilterControl.GyroFilterMode(1)
        mag_mode  = Registers.ImuFilterControl.MagFilterMode(1)
        filter.accelWindowSize = 10
        filter.gyroWindowSize = 10
        filter.magWindowSize = 4
        filter.accelFilterMode = accel_mode
        filter.gyroFilterMode = gyro_mode
        filter.magFilterMode = mag_mode
        self.vs.writeRegister(filter)
        print("low-pass filter configured.")

        
    def read_sensor_data(self):
        """ Read and return sensor data, optionally bias-corrected. """
        self.vs.readRegister(self.ypr_mar_register)
        imu_data = {
            'Yaw': self.ypr_mar_register.yaw,
            'Pitch': self.ypr_mar_register.pitch,
            'Roll': self.ypr_mar_register.roll,
            'AccelX': self.ypr_mar_register.accelX,
            'AccelY': self.ypr_mar_register.accelY,
            'AccelZ': self.ypr_mar_register.accelZ,
            'MagX': self.ypr_mar_register.magX,
            'MagY': self.ypr_mar_register.magY,
            'MagZ': self.ypr_mar_register.magZ,
            'GyroX': self.ypr_mar_register.gyroX,
            'GyroY': self.ypr_mar_register.gyroY,
            'GyroZ': self.ypr_mar_register.gyroZ,
        }

        # Apply bias correction if available
        if hasattr(self, 'biases'):
            for key in self.biases:
                imu_data[key] -= self.biases[key]

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



    def visualize_ypr_and_imu(self, duration=20, rate=80):
        from pyqtgraph.Qt import QtCore, QtWidgets
        import pyqtgraph as pg
        import numpy as np

        app = QtWidgets.QApplication([])

        win = pg.GraphicsLayoutWidget(show=True, title="VN100 Real-Time IMU Visualization")
        win.resize(1200, 800)

        data_len = int(duration * rate)
        x = np.linspace(-data_len, 0, data_len)

        # Create plots
        plots = {}
        curves = {}
        sensors = {
            "YPR": ("Yaw", "Pitch", "Roll"),
            "Accel": ("AccelX", "AccelY", "AccelZ"),
            "Mag": ("MagX", "MagY", "MagZ"),
            "Gyro": ("GyroX", "GyroY", "GyroZ")
        }

        for i, (label, channels) in enumerate(sensors.items()):
            p = win.addPlot(title=label)
            p.addLegend()
            plots[label] = p
            curves[label] = {}
            for j, axis in enumerate(channels):
                color = ['y', 'r', 'b'][j]
                curves[label][axis] = p.plot(pen=color, name=axis)
            if i < len(sensors) - 1:
                win.nextRow()

        # Initialize data
        data_buffers = {label: {axis: np.zeros(data_len) for axis in axes} for label, axes in sensors.items()}

        def update():
            imu = self.read_sensor_data()

            for label, axes in sensors.items():
                for axis in axes:
                    buffer = data_buffers[label][axis]
                    buffer[:] = np.roll(buffer, -1)
                    buffer[-1] = imu[axis]
                    curves[label][axis].setData(x, buffer)

        timer = QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(int(1000 / rate))
        app.exec_()

    def logger(self, duration=5):
        """
        Method to log sensor data to a file for a given duration.
        Args:
            file_path (str): The file path to save the log.
            duration (float): The duration to log data (in seconds).
        """
        csvExporter = ExporterCsv("csv_output", True)
        self.vs.subscribeToMessage(
            csvExporter.getQueuePtr(),
            vectornav.Registers.BinaryOutputMeasurements(),
            vectornav.FaPacketDispatcher.SubscriberFilterType.AnyMatch
        )
        
        self.vs.subscribeToMessage(
            csvExporter.getQueuePtr(),
            "VN",
            vectornav.AsciiPacketDispatcher.SubscriberFilterType.StartsWith
        )

        csvExporter.start()
        print("Logging to ", "csv_output")
        
        time.sleep(duration)

        csvExporter.stop()
        print("ExportFromSensor example complete.")


        
    def disconnect(self):
        """ Disconnect from the sensor. """
        self.vs.disconnect()
        print("Disconnected successfully.")



# Usage Example
if __name__ == "__main__":
    try:
        sensor = VN100(port="COM7",baudrate=115200)
        sensor.connect()
        # sensor.configure_baud_rate(230400)
        # sensor.configure_initial_heading(0.0)
        sensor.configure_async_output()
        sensor.configure_async_output_freq()
        sensor.configure_vpe()
        sensor.configure_filter()

        # sensor.configure_bias()

        # sensor.stream_data(duration=20, rate=80)
        # sensor.visualize_ypr_and_imu(duration=20, rate=80)
        sensor.logger(file_path="log.bin", duration=5)
        # export_bin_to_csv("log.bin", "csv_output", with_timestamps=True)
        sensor.disconnect()
    except Exception as e:
        sensor.disconnect()
        print(e)
        traceback.print_exc()  # This prints the full traceback


