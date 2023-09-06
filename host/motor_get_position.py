import time

import recoil
import util


TRANSPORT = util.getTransport()
DEVICE_ID = util.getID()

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
