import time

import serial
import can
import can.interfaces.serial

import recoil

CURRENT_DEVICE_ID = 1
TARGET_DEVICE_ID = 2

transport = recoil.SerialCANTransport(port="/dev/ttyACM0", baudrate=115200)
transport.start()

motor = recoil.MotorController(transport, device_id=CURRENT_DEVICE_ID)

print(motor.getInfo())

print(motor.setID(TARGET_DEVICE_ID))


motor = recoil.MotorController(transport, device_id=TARGET_DEVICE_ID)

print(motor.getInfo())

motor.storeSettingToFlash()

transport.stop()
