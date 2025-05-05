from src.VN100 import VN100
import time
imu = VN100(port='/dev/ttyUSB0', baudrate=115200, hz=10)

imu.set_mode(enable=1, headingMode=1, filteringMode=1, tuningMode=1)