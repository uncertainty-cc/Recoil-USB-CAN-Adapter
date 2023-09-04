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

# print(motor._readParameterFloat(recoil.Command.POSITION_KP)[1])
# motor._writeParameterFloat(recoil.Command.POSITION_KP, 2)
# print(motor._readParameterFloat(recoil.Command.POSITION_KP)[1])

# print(motor._readParameterFloat(recoil.Command.VELOCITY_KP)[1])
# motor._writeParameterFloat(recoil.Command.VELOCITY_KP, 0.15)
# print(motor._readParameterFloat(recoil.Command.VELOCITY_KP)[1])

print(motor._readParameterFloat(recoil.Command.TORQUE_LIMIT)[1])
motor._writeParameterFloat(recoil.Command.TORQUE_LIMIT, 1)
print(motor._readParameterFloat(recoil.Command.TORQUE_LIMIT)[1])

motor.storeSettingToFlash()

transport.stop()
