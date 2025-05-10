from vectornav import Sensor, Registers

import time
vs = Sensor()

vs.connect("COM7", Sensor.BaudRate.Baud115200)
if not vs.verifySensorConnectivity():
    raise Exception("Wrong baud rate or incorrect port")
if not vs.verifySensorConnectivity():
    vs.autoConnect("COM7")

# set initialheaidng
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

#
vpeControl = Registers.VpeBasicControl()
vpeControl.headingMode = Registers.VpeBasicControl.HeadingMode.Absolute
vpeControl.filteringMode = Registers.VpeBasicControl.FilteringMode.AdaptivelyFiltered
vpeControl.tuningMode = Registers.VpeBasicControl.TuningMode.Adaptive

#
yprRegister = Registers.YawPitchRoll()
qmarRegister = Registers.QuatMagAccelRate()
yprMarRegister = Registers.YprMagAccelAngularRates()

startime = time.time()
while(time.time()-startime <8):
    # vs.readRegister(yprRegister)
    # print(f"Current Reading: Yaw - {yprRegister.yaw}, Pitch - {yprRegister.pitch}, Roll - {yprRegister.roll} ")

    # vs.readRegister(qmarRegister)
    # print(f"acc_x - {qmarRegister.accelX}, acc_y - {qmarRegister.accelY}, acc_z - {qmarRegister.accelZ}")

    vs.readRegister(yprMarRegister)
    print(f"Current Reading: Yaw - {yprMarRegister.yaw}, Pitch - {yprMarRegister.pitch}, Roll - {yprMarRegister.roll}, accZ - {yprMarRegister.accelZ} ")

vs.reset()
print("reseting ...")
time.sleep(2)
print("reset success")

startime = time.time()
while(time.time()-startime <3):
    # vs.readRegister(yprRegister)
    # print(f"Current Reading: Yaw - {yprRegister.yaw}, Pitch - {yprRegister.pitch}, Roll - {yprRegister.roll} ")

    # vs.readRegister(qmarRegister)
    # print(f"acc_x - {qmarRegister.accelX}, acc_y - {qmarRegister.accelY}, acc_z - {qmarRegister.accelZ}")

    vs.readRegister(yprMarRegister)
    print(f"Current Reading: Yaw - {yprMarRegister.yaw}, Pitch - {yprMarRegister.pitch}, Roll - {yprMarRegister.roll}, accZ - {yprMarRegister.accelZ} ")



vs.disconnect()
print("exit successfully")