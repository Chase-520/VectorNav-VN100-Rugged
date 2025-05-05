import serial
import threading
from threading import Thread, Event
import time
import numpy as np
import csv

class VN100:
    def __init__(self, port='COM7', baudrate=115200, timeout=1, hz = 10):
        """
        Initialize the VN-100 IMU on the specified serial port.
        :param port: Serial port to which the VN-100 is connected (e.g., 'COM7').
        :param baudrate: Baud rate for the serial connection (default is 115200).
        :param timeout: Timeout for the serial connection (default is 1 second).
        :param frequency: Frequency of data read (default is 10Hz).
        """
        self.__ser = serial.Serial(port, baudrate, timeout=timeout)
        self.__RREG = self._init_register_map()
        self.__thread = Thread(target=self.read_thread)
        self.__hz = hz
        self.__stop_event = Event()

    def _write(self, cmd: bytes):
        """Write a command to the sensor"""
        self.__ser.write(cmd)

    def _read_response(self):
        return self.__ser.readline()

    def get_serial_number(self):
        """
        Get the sensor's serial number.
        """
        self.__ser.reset_input_buffer()
        cmd = b'$VNRRG,03*XX\r\n'
        self._write(cmd)
        x = self._read_response()
        x = str(x.strip()).split("*")[0].split(',')[-1]
        print("Model number is:", x)
        return x

    def model_name(self):
        """
        Get the sensor model name.
        """
        cmd = b'$VNRRG,1*9527\r'
        self._write(cmd)
        x = self._read_response()
        x = str(x.strip()).split("*")[0].split(',')[-1]
        print("Model name:", x)
        return x

    def stop_async(self):
        """
        Stop asynchronously sent data so that the user can send commands
        and get responses back without worrying about broadcasted values.
        (For both UART channels)
        """
        cmd1 = b'$VNWRG,7,0,1*40\r'
        cmd2 = b'$VNWRG,7,0,2*XX\r'
        self._write(cmd1)
        self._write(cmd2)

    def set_mode(self, enable=1, headingMode=1, filteringMode=1, tuningMode=1):
        """
        Set the mode of the sensor.
        :param mode: List of modes to set.
        ["enable", "headingMode", "filteringMode", "tuningMode"]
        enable	Enables or disables VPE	    0 = disable,    1 = enable
        headingMode	Sets heading mode	    0 = absolute,   1 = relative,   2 = inertial
        filteringMode	Enables filtering	0 = off,        1 = on
        tuningMode	Enables dynamic tuning	0 = off,        1 = on
        """
        self.__ser.reset_input_buffer()
        cmd = b"$VNWRG,35,{enable},{headingMode},{filteringMode},{tuningMode}*XX\r\n"
        self._write(cmd)
        print(self._read_response())


    def get_imu_data(self):
        """
        Read IMU data (Register 54) using the read_register method.
        """
        return self.read_register(9)


    def read_register(self, reg_number: int):
        """
        Read a specified register and print the results.
        """
        self.__ser.reset_input_buffer()
        cmd = b'$VNRRG,' + str(reg_number).encode("ascii") + b'*XX\r\n'
        self._write(cmd)
        x = self._read_response()
        print(x)
        x = str(x.strip()).split("*")[0].split(',')[1:]
        regs = self.__RREG.get(reg_number, [None, []])[1]
        for i, val in enumerate(x):
            print(f"{regs[i]}: {val}")
        return dict(zip(regs, x))

    def magnetic_disturbance(self, value=0):
        """
        Send command to indicate magnetic disturbance.
        """
        cmd = b'$VNKMD,' + str(value).encode("ascii") + b'*47\r\n'
        self._write(cmd)
        print(self._read_response())

    def accel_disturbance(self, value=0):
        """
        Send command to indicate accelerometer disturbance.
        """
        cmd = b'$VNKAD' + str(value).encode("ascii") + b'*48\r\n'
        self._write(cmd)
        print(self._read_response())

    def read_thread(self, reg_number=54, callback=None):
        """
        Thread target function that continuously reads async IMU data.
        """
        while not self.__stop_event.is_set():
            try:
                data = self.get_imu_data()
                if callback:
                    data = callback(data)

                #print(data)

                time.sleep(1/self.__hz)
            except Exception as e:
                print(f"IMU read error: {e}")


    def start(self):
        self.__thread.start()

    def stop(self):
        """
        Stop the read thread.
        """
        self.__stop_event.set()
        self.__thread.join()
        self.__ser.close()

    def _init_register_map(self):
        """
        Register number and format of data used.
        Comments retained from original code.
        """
        RREG = {}
        RREG.update({8:["yaw pitch roll",['Yaw','Pitch',"Roll"]]})
        RREG.update({9:["Attitude Quaternion",["Quat[0]","Quat[1]","Quat[2]","Quat[3]"]]})
        RREG.update({27:["Yaw,pitch,roll,Magnetic,Acceleration,and Angular Rates [YMR]",['Yaw','Pitch','Roll','MagX',"MagY","MagZ","AccelX","AccelY","AccelZ","GyroX","GyroY","GyroZ"]]})
        RREG.update({15:["QMR",["Quat[0]","Quat[1]","Quat[2]","Quat[3]","MagX","MagY","MagZ","AccelX","AccelY","AccelZ","GyroX","GyroY","GyroZ"]]})
        RREG.update({17:["Magnetic Measurements",["Magx","MagY","MagZ"]]})
        RREG.update({18:["Acceleration Measurements",["AccelX","AccelY","AccelZ"]]})
        RREG.update({19:["Compensated angular Rate Measurements",["GyroX","GyroY","GyroZ"]]})
        RREG.update({20:["Magnetic,Acceleration, AND angular RATES",["Magx","MagY","MagZ","AccelX","AccelY","AccelZ","GyroX","GyroY","GyroZ"]]})
        # Configuration registers (read/write)
        RREG.update({35:["Configuration registers",["enable","headingMode","filteringMode","tuningMode"]]})
        RREG.update({36:["VPE Magnetometer Basic Tuning",['BaseTuningX',"BaseTuningY","BaseTuningZ","AdaptiveTuningx","AdaptiveTuningY","AdaptiveTuningZ"]]})
        RREG.update({38:["VPE Acceleration Basic tuning",['BaseTuningX',"BaseTuningY","BaseTuningZ","AdaptiveTuningx","AdaptiveTuningY","AdaptiveTuningZ","AdaptiveFilteringX","AdaptiveFilteringY","AdaptiveFilteringZ"]]})
        RREG.update({43:["Filter Startup Gyro Bias",["X-axis Gyro Bias Estimate","Y-axis Gyro Bias Estimate","Z-axis Gyro Bias Estimate"]]})
        RREG.update({40:["VPE Gyro Basic Tuning",["VAngularWalkX","VAngularWalkY","VAngularWalkZ","BaseTuningX","BaseTuningY","BaseTuningZ","AdaptiveTuningX","AdaptiveTuningY","AdaptiveTuningZ"]]})
        RREG.update({54:["IMU",["Yaw","Pitch","Row","Magx","MagY","MagZ","AccelX","AccelY","AccelZ","GyroX","GyroY","GyroZ"]]})
        RREG.update({80:["IMU",["Yaw","Pitch","Row","Magx","MagY","MagZ","AccelX","AccelY","AccelZ","GyroX","GyroY","GyroZ"]]})
        #RREG.update({80:["Delta theta and velocity",["DeltaTime","DeltaThetaX","DeltaThetaY","DeltaThetaZ","DeltaVelocitx","DeltaVolocityY","DeltaVolocityZ"]]})
        # Hard/soft iron estimator
        RREG.update({44:["Magnetometer Calibration control",["HSIMode","HSIOutput","ConvergeRate"]]})
        RREG.update({47:["Calculated Magnetometer Calibration",["C[0,0]","C[0,1]","C[0,2]","C[1,0]","C[1,1]","C[1,2]","C[2,0]","C[2,1]","C[2,2]","B[0]","B[1]","B[2]"]]})
        RREG.update({51:["Velocity compansation control",["Mode","VelocityTuning","RateTuning"]]})
        RREG.update({50:["Velocity Compansation Measurements",["VelocityX","VelocitY","VelocityZ"]]})
        RREG.update({21:["Magnetic and gravity reference vectors",["MagRefX","MagRefY","MagRefZ","AccRefX","AccRefY","AccRefZ"]]})
        RREG.update({83:["Reference Vector Configuration",["UseMagModel","UseGravityModel","Resv","Resv","RecalcThreshold","Year","Latitude","Longitude","Altitude"]]})
        RREG.update({82:["Delta Theta and Delta velocity configuration",["IntegrationFrame","GyroCompensation","AccelCompensation","Reserved","Reserved"]]})
        RREG.update({85:["IMU filtering Configuration",["MagWindowSize","AccelWindowSize","GryoWindowSize","TempWindowSize","PresWindowSize","MagFilterMode","AccelFilterMode","GyroFilterMode","TempFilterMode","PresFilterMode"]]})
        RREG.update({26:["C[0,0]","C[0,1]",["C[0,1]","C[0,2]","C[1,0]","C[1,1]","C[1,2]","C[2,0]","C[2,1]","C[2,2]"]]})
        RREG.update({84:["Gyro Compensation",["C[0,0]","C[0,1]","C[0,2]","C[1,0]","C[1,1]","C[1,2]","C[2,0]","C[2,1]","C[2,2]","B[0]","B[1]","B[2]"]]})
        RREG.update({23:["Magnetic Compensation",["C[0,0]","C[0,1]","C[0,2]","C[1,0]","C[1,1]","C[1,2]","C[2,0]","C[2,1]","C[2,2]","B[0]","B[1]","B[2]"]]})
        RREG.update({25:["Acceleration Compensation",["C[0,0]","C[0,1]","C[0,2]","C[1,0]","C[1,1]","C[1,2]","C[2,0]","C[2,1]","C[2,2]","B[0]","B[1]","B[2]"]]})
        return RREG
