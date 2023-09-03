import time

import serial
import can
import can.interfaces.serial

import recoil
import util


TRANSPORT = util.getTransport()
DEVICE_ID = 1

transport = recoil.SerialCANTransport(port=TRANSPORT, baudrate=115200)
transport.start()

motor = recoil.MotorController(transport, device_id=DEVICE_ID)

motor.setMode(recoil.Mode.IDLE)

transport.stop()
