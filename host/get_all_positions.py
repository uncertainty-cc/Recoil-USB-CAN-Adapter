import time

import serial
import can
import can.interfaces.serial

import recoil
import util


TRANSPORT = util.getTransport()

transport = recoil.SerialCANTransport(port=TRANSPORT, baudrate=115200)
transport.start()

motor_2 = recoil.MotorController(transport, device_id=2)
motor_4 = recoil.MotorController(transport, device_id=4)
motor_6 = recoil.MotorController(transport, device_id=6)
motor_8 = recoil.MotorController(transport, device_id=8)
motor_10 = recoil.MotorController(transport, device_id=10)
motor_12 = recoil.MotorController(transport, device_id=12)

positions = [0] * 16

try:
    while True:
        positions[2] = motor_2.getPositionMeasured()
        positions[4] = motor_4.getPositionMeasured()
        positions[6] = motor_6.getPositionMeasured()
        positions[8] = motor_8.getPositionMeasured()
        positions[10] = motor_10.getPositionMeasured()
        positions[12] = motor_12.getPositionMeasured()
        
        print(positions)
        time.sleep(0.02)
        
except KeyboardInterrupt:
    transport.stop()
