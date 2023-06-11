import time

import serial
import can
import can.interfaces.serial

import recoil

# TRANSPORT = "/dev/ttyACM0"
TRANSPORT = "COM14"
# TRANSPORT = "COM32"

transport = recoil.SerialCANTransport(port=TRANSPORT, baudrate=115200)
transport.start()

try:
    while True:
        motor = recoil.MotorController(transport, device_id=1)

        print("ping id {id}: ".format(id=1), motor.ping())

except KeyboardInterrupt:
    pass

transport.stop()
