import time

import serial
import can
import can.interfaces.serial

import recoil
import util


TRANSPORT = util.getTransport()
DEVICE_ID = util.getID()

transport = recoil.SerialCANTransport(port=TRANSPORT, baudrate=115200)
transport.start()

motor = recoil.MotorController(transport, device_id=DEVICE_ID)

print(motor._readParameterFloat(recoil.Command.VELOCITY_KP)[1])
motor._writeParameterFloat(recoil.Command.VELOCITY_KP, 0.03)
print(motor._readParameterFloat(recoil.Command.VELOCITY_KP)[1])

motor.storeSettingToFlash()

transport.stop()
