from vectornav import Sensor, Registers
import time
import sys

FIELDS = ['Yaw', 'Pitch', 'Roll', 'AccelX', 'AccelY', 'AccelZ', 'MagX', 'MagY', 'MagZ', 'GyroX', 'GyroY', 'GyroZ']

vs = Sensor()
vs.connect("COM7", Sensor.BaudRate.Baud115200)

if not vs.verifySensorConnectivity():
    raise Exception("Wrong baud rate or incorrect port")

vs.setInitialHeading(0.0)

modelRegister = Registers.Model()
vs.readRegister(modelRegister)
print(f"Sensor Model Number: {modelRegister.model}")

# Configure async output type
asyncDataOutputType = Registers.AsyncOutputType()
asyncDataOutputType.ador = Registers.AsyncOutputType.Ador.YPR
asyncDataOutputType.serialPort = Registers.AsyncOutputType.SerialPort.Serial1
vs.writeRegister(asyncDataOutputType)
print(f"ADOR Configured")

asyncDataOutputFreq= Registers.AsyncOutputFreq()
asyncDataOutputFreq.adof = Registers.AsyncOutputFreq.Adof.Rate100Hz
asyncDataOutputFreq.serialPort = Registers.AsyncOutputFreq.SerialPort.Serial1
vs.writeRegister(asyncDataOutputFreq)
print("ADOF Configured")

# Configure VPE
vpeControl = Registers.VpeBasicControl()
vpeControl.headingMode = Registers.VpeBasicControl.HeadingMode.Relative
vpeControl.filteringMode = Registers.VpeBasicControl.FilteringMode.AdaptivelyFiltered
vs.writeRegister(vpeControl)
print(f"VPE Control Configured")

# filter
filter = Registers.ImuFilterControl()
filter.accelWindowSize = 10
filter.gyroWindowSize = 10
filter.magWindowSize = 4
gyro_mode = Registers.ImuFilterControl.GyroFilterMode(1)
gyro_mode.comp = 1  # Enable 'comp' mode (1 = true/on)
filter_control = Registers.ImuFilterControl()
filter_control.gyroFilterMode = gyro_mode  # Assign the configured mode
vs.writeRegister(filter)
# Prepare register
yprMarRegister = Registers.YprMagAccelAngularRates()

# Start live printing
print("\nStreaming IMU data (Press Ctrl+C to stop):\n")
try:
    while True:
        vs.readRegister(yprMarRegister)
        imu_data = {
            'Yaw': f"{yprMarRegister.yaw:.2f}",
            'Pitch': f"{yprMarRegister.pitch:.2f}",
            'Roll': f"{yprMarRegister.roll:.2f}",
            'AccelX': f"{yprMarRegister.accelX:.3f}",
            'AccelY': f"{yprMarRegister.accelY:.3f}",
            'AccelZ': f"{yprMarRegister.accelZ:.3f}",
            'MagX': f"{yprMarRegister.magX:.2f}",
            'MagY': f"{yprMarRegister.magY:.2f}",
            'MagZ': f"{yprMarRegister.magZ:.2f}",
            'GyroX': f"{yprMarRegister.gyroX:.3f}",
            'GyroY': f"{yprMarRegister.gyroY:.3f}",
            'GyroZ': f"{yprMarRegister.gyroZ:.3f}",
            # 'pos_x':f"{GnssCompassBaseline.positionX:.3f}",
            # 'pos_y':f"{GnssCompassBaseline.positionY:.3f}",
            # 'pos_z':f"{GnssCompassBaseline.positionZ:.3f}"

        }

        output = ' | '.join(f'{k}: {imu_data[k]}' for k in FIELDS)
        sys.stdout.write(f'\r{output.ljust(140)}')
        sys.stdout.flush()

        time.sleep(0.05)  # Small delay to avoid spamming output too fast

except KeyboardInterrupt:
    print("\n\nStreaming stopped by user.")

vs.disconnect()
print("Disconnected successfully.")
