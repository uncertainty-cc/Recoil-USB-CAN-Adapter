import time

import serial
import can
import can.interfaces.serial

import recoil
import util


TRANSPORT = util.getTransport()

CURRENT_DEVICE_ID = 1
TARGET_DEVICE_ID = 2

transport = recoil.SerialCANTransport(port=TRANSPORT, baudrate=115200)
transport.start()

motor = recoil.MotorController(transport, device_id=CURRENT_DEVICE_ID)

print(motor.getInfo())

print(motor.setID(TARGET_DEVICE_ID))


motor = recoil.MotorController(transport, device_id=TARGET_DEVICE_ID)

print(motor.getInfo())

motor.storeSettingToFlash()

transport.stop()
