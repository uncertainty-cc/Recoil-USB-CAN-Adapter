import time
import math
import threading

from robot import Humanoid
from cc.xboxcontroller import XboxController, Hand, Button
from command_scheduler import Scheduler, Command, WaitCommand, LogCommand, EaseInOutQuadTrajectoryCommand

import util


TRANSPORT = util.getTransport()

robot = Humanoid(transport=TRANSPORT)
stick = XboxController(0, deadzone=0.)

active_joint = 0

def onButton(button, pressed):
    global active_joint
    if not pressed:
        return
    if button != Button.DPAD_L and button != Button.DPAD_R:
        return
    
    if button == Button.DPAD_L:
        if active_joint > 0:
            active_joint -= 1
    if button == Button.DPAD_R:
        if active_joint < (len(robot.joints) - 1):
            active_joint += 1
    print("current active controller: {}".format(robot.joints[active_joint].device_id))


scheduler = Scheduler()

# 8 ~= 30.5 deg
# 12 ~= 45.9  deg
# 16 ~= 61.1 deg
# 24 ~= 91.7 deg

scheduler.addCommand(LogCommand("Starting trajectory!!"))
scheduler.addCommand(WaitCommand(1))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, 0, 12, 4))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, 12, 0, 4))
scheduler.addCommand(WaitCommand(1))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, 0, 12, 4))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, 12, 0, 4))
scheduler.addCommand(WaitCommand(1))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, 0, 11, 2))
scheduler.addCommand(WaitCommand(1))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, 11, 0, 2))

robot.start()

robot.resetTargetPositions()

stick.onButton = onButton

trajectory_control = False

start_pos = 0

print("Press A to start position control")

while not robot.is_stopped.is_set():
    try:
        stick.update()
        robot.update()
    
        if stick.getXButton():
            robot.setDamping()
            robot.stop()
        elif stick.getAButton():
            # blocking
            robot.rampUp(torque_limit=0.75)
        elif stick.getBButton():
            trajectory_control = True
            
            scheduler.start()

        if trajectory_control and not scheduler.isFinished():
            scheduler.update()
            
            print("trajectory in progress", robot.position_targets[0])

        else:
            if trajectory_control:
                print("trajectory done")
                trajectory_control = False
            robot.position_targets[active_joint] += 0.2 * stick.axes["LY"]
            

        time.sleep(0.01)
            
    except KeyboardInterrupt:
        robot.stop()
        print("keyboard interrupted")

print("Dampen mode")

robot.setDamping()

# add a 0.5 second delay to allow button debouncing
for i in range(50):
    stick.update()
    robot.update()
    time.sleep(0.01)

print("Press X again to shut down")

while not stick.getXButton():
    try:
        stick.update()
        robot.update()
    except KeyboardInterrupt:
        break


robot.setIdle()
robot.setIdle()
print("robot stopped")

robot.transport.stop()

