import time

from robot import Humanoid
from keyboardcontroller import KeyboardController
import util


TRANSPORT = util.getTransport()

robot = Humanoid(transport=TRANSPORT)
keycontroller = KeyboardController()

active_joint = 0


def onLeftArrowPressed():
    global active_joint
    if active_joint > 0:
        active_joint -= 1
    print("current active controller: {}".format(robot.joints[active_joint].device_id))

def onRightArrowPressed():
    global active_joint
    if active_joint < (len(robot.joints) - 1):
        active_joint += 1
    print("current active controller: {}".format(robot.joints[active_joint].device_id))

def onAPressed():
    robot.rampUp()

def onXPressed():
    robot.setDamping()
    robot.stop()

keycontroller.onLeftArrowPressed = onLeftArrowPressed
keycontroller.onRightArrowPressed = onRightArrowPressed

keycontroller.onAPressed = onAPressed
keycontroller.onXPressed = onXPressed

robot.start()
keycontroller.start()

robot.resetTargetPositions()


while not robot.is_stopped.is_set():
    try:
        robot.update()
    
        # robot.target_positions[active_joint] += 0.2 * stick.axes["LY"]
        robot.printPositions()

        time.sleep(0.01)
            
    except KeyboardInterrupt:
        robot.stop()
        print("keyboard interrupted")

print("Dampen mode")

robot.setDamping()

# add a 1 second delay to allow button debouncing
for i in range(100):
    robot.update()
    time.sleep(0.01)

print("Press Control-C again to shut down")

while True:
    try:
        robot.update()
    except KeyboardInterrupt:
        break


robot.setIdle()
robot.setIdle()
print("robot stopped")

robot.transport.stop()
keycontroller.stop()
