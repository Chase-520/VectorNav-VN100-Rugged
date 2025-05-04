from src.VN100 import VN100
import time
imu = VN100(port='/dev/ttyUSB0', baudrate=115200, hz=10)

imu.start()

for i in range(10):
    time.sleep(1)

imu.stop()