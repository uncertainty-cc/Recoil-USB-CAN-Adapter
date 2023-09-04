import time

import can.interfaces.serial

import recoil
import util


TRANSPORT = util.getTransport()

transport = recoil.SerialCANTransport(port=TRANSPORT, baudrate=115200)
transport.start()

try:
    for i in range(1, 1<<5):
        motor = recoil.MotorController(transport, device_id=i)

        print("ping id {id}: ".format(id=i), motor.ping())

        time.sleep(0.02)
except KeyboardInterrupt:
    pass

transport.stop()
