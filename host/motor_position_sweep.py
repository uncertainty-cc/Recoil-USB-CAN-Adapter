import time
import math

import recoil
import util

TRANSPORT = util.getTransport()
DEVICE_ID = util.getID()

# SWEEP_MODE = "sine"
SWEEP_MODE = "square"

SWEEP_FREQ = 1
SWEEP_MAGNITUDE = 10 * math.pi

transport = recoil.SerialCANTransport(port=TRANSPORT, baudrate=115200)
transport.start()

motor = recoil.MotorController(transport, device_id=DEVICE_ID)

print("set mode to IDLE")
motor.setMode(recoil.Mode.IDLE)
time.sleep(0.1)

print("set position to 0")
target_position = 0
motor.setPositionTarget(target_position)
time.sleep(0.1)

print("enable Motor")
motor.setMode(recoil.Mode.POSITION)

t = time.time()


try:
    while True:
        if SWEEP_MODE == "sine":
            target_position = SWEEP_MAGNITUDE * (math.sin(SWEEP_FREQ * time.time()))
        else:    
            if (SWEEP_FREQ * time.time()) % 2 < 1:
                target_position = .5 * SWEEP_MAGNITUDE
            else:
                target_position = -.5 * SWEEP_MAGNITUDE
        
        motor.setPositionTarget(target_position)
        
        # print("target: {0}, \tmeasured: {1}".format(target_position, motor.getPositionMeasured()))

        motor.feed()
        # time.sleep(0.01)

except KeyboardInterrupt:
    print("Stopping motor")
    motor.setMode(recoil.Mode.IDLE)

transport.stop()
