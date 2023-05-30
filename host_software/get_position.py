import time

import serial
import can
import can.interfaces.serial

import recoil

transport = recoil.SerialCANTransport(port="/dev/ttyACM0", baudrate=115200)
transport.start()

motor = recoil.MotorController(transport, device_id=1)

try:
    while True:
        pos = motor.getPositionMeasured()
        print(pos)
        time.sleep(0.02)
        
except KeyboardInterrupt:
    transport.stop()
