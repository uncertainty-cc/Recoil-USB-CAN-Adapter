import time

import serial
import can
import can.interfaces.serial

import recoil
import argparser


TRANSPORT = "/dev/ttyACM0"
DEVICE_ID = argparser.getID()

transport = recoil.SerialCANTransport(port=TRANSPORT, baudrate=115200)
transport.start()

motor = recoil.MotorController(transport, device_id=DEVICE_ID)

try:
    while True:
        pos = motor.getPositionMeasured()
        print(pos)
        time.sleep(0.02)
        
except KeyboardInterrupt:
    transport.stop()
