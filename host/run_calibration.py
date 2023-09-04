import time
import math

import recoil
import util

TRANSPORT = util.getTransport()
DEVICE_ID = util.getID()

transport = recoil.SerialCANTransport(port=TRANSPORT, baudrate=115200)
transport.start()

motor = recoil.MotorController(transport, device_id=DEVICE_ID)

print("set mode to IDLE")
motor.setMode(recoil.Mode.IDLE)
time.sleep(0.1)

print("set mode to calibration")
motor.setMode(recoil.Mode.CALIBRATION)
time.sleep(0.1)

try:
    while True:
        motor.feed()
        
        if motor.getMode() == recoil.Mode.IDLE:
            print("Calibration complete")
            break
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping motor")
    motor.setMode(recoil.Mode.IDLE)

transport.stop()
