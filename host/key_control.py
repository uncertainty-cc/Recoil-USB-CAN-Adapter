import time
import math
import threading

import numpy as np

from keyboardcontroller import KeyboardController
import recoil
import util
from command_scheduler import Scheduler, Command, WaitCommand, LogCommand, EaseInOutQuadTrajectoryCommand


TRANSPORT = util.getTransport()

class Humanoid:
    N_JOINTS = 12
    def __init__(self, transport):
        self.transport = recoil.SerialCANTransport(port=transport, baudrate=115200)
        self.is_stopped = threading.Event()
        
        self.joints = [
            recoil.MotorController(self.transport, device_id=1),
            recoil.MotorController(self.transport, device_id=2),
            recoil.MotorController(self.transport, device_id=3),
            recoil.MotorController(self.transport, device_id=4),
            recoil.MotorController(self.transport, device_id=5),
            recoil.MotorController(self.transport, device_id=6),
            recoil.MotorController(self.transport, device_id=7),
            recoil.MotorController(self.transport, device_id=8),
            recoil.MotorController(self.transport, device_id=9),
            recoil.MotorController(self.transport, device_id=10),
            recoil.MotorController(self.transport, device_id=11),
            recoil.MotorController(self.transport, device_id=12),
        ]
        
        self.torque_limit = 0.1

        self.position_targets = [0] * Humanoid.N_JOINTS
        self.position_measureds = [0] * Humanoid.N_JOINTS
        self.position_offsets = [0] * Humanoid.N_JOINTS

    def start(self):
        self.transport.start()
        self.is_stopped.clear()

    def stop(self):
        self.is_stopped.set()
        print("stopping...")

    def resetTargetPositions(self):
        for i, j in enumerate(self.joints):
            self.position_offsets[i] = j.getPositionMeasured()
            self.position_targets[i] = 0
        self.printPositions()
        
    def printPositions(self):
        for i, j in enumerate(self.joints):
            print("{0}: {1:.3f}".format(j.device_id, self.position_targets[i]), end="\t")
        print()

    def setDamping(self):                
        for j in self.joints:
            j.setMode(recoil.Mode.DAMPING)
        
    def setIdle(self):
        for j in self.joints:
            j.setMode(recoil.Mode.IDLE)

    def rampUp(self, torque_limit):
        self.torque_limit = torque_limit
        print("ramping up position control...")
        
        for i, j in enumerate(self.joints):
            j.setPositionTarget(self.position_targets[i] + self.position_offsets[i])
            j.setTorqueLimit(0)
            j.setMode(recoil.Mode.POSITION)

        ramp_up_time_ms = 1
        dt = 0.01
        n_counts = int(ramp_up_time_ms / dt)
        for t in range(n_counts):
            try:
                if self.is_stopped.is_set():
                    return
                ramp_torque_limit = self.torque_limit * (t / n_counts)

                for i, j in enumerate(self.joints):
                    j.setPositionTarget(self.position_targets[i] + self.position_offsets[i])
                    j.setTorqueLimit(ramp_torque_limit)
                    j.feed()
                        
                print("Booting up...\t {:.3f} / {:.3f}".format(ramp_torque_limit, self.torque_limit))

                time.sleep(dt)
            except KeyboardInterrupt:
                self.stop()
                continue
    
    def update(self):
        for i, j in enumerate(self.joints):
            self.position_measureds[i] = j.getPositionMeasured() - self.position_offsets[i]
            j.setPositionTarget(self.position_targets[i] + self.position_offsets[i])
            j.feed()


robot = Humanoid(transport=TRANSPORT)
keycontroller = KeyboardController()

active_joint = 0


scheduler = Scheduler()

# 8 ~= 30.5 deg
# 12 ~= 45.9  deg
# 16 ~= 61.1 deg
# 24 ~= 91.7 deg

zero_pos = np.zeros(Humanoid.N_JOINTS)
left_raise = np.array([1, 0, 0, 0, 0, 0, -2, 0, 0, 0, -1, 0]) * 10
right_raise = np.array([0, -1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1]) * 10

bend_knee = np.array([1, -1, 0, 0, 0, 0, -2, 2, 0, 0, -1, 1]) * 2

left_kick = np.array([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0]) * 8
right_kick = np.array([0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]) * 8

t = 0.5

scheduler.addCommand(LogCommand("Starting trajectory!!"))
scheduler.addCommand(WaitCommand(1))

''' step motion
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, zero_pos, bend_knee, t))
scheduler.addCommand(WaitCommand(1))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, bend_knee, bend_knee+left_kick-right_kick, t))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, bend_knee+left_kick-right_kick, bend_knee+-left_kick+right_kick, t))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, bend_knee+-left_kick+right_kick, bend_knee+left_kick-right_kick, t))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, bend_knee+left_kick-right_kick, bend_knee+-left_kick+right_kick, t))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, bend_knee+-left_kick+right_kick, bend_knee+left_kick-right_kick, t))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, bend_knee+left_kick-right_kick, bend_knee+-left_kick+right_kick, t))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, bend_knee+-left_kick+right_kick, bend_knee+left_kick-right_kick, t))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, bend_knee+left_kick-right_kick, zero_pos, t))
'''


scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, zero_pos, left_raise, t))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, left_raise, right_raise, t))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, right_raise, left_raise, t))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, left_raise, right_raise, t))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, right_raise, left_raise, t))
scheduler.addCommand(EaseInOutQuadTrajectoryCommand(robot, left_raise, zero_pos, t))

robot.start()
keycontroller.start()

robot.resetTargetPositions()

trajectory_control = False

start_pos = 0

print("Press A to ramp up")
print("Press B to start position control")

while not robot.is_stopped.is_set():
    try:
        robot.update()

        if keycontroller.get("Q") or keycontroller.get("X"):
            robot.setDamping()
            robot.stop()
        elif keycontroller.get("A"):
            # blocking
            robot.rampUp(torque_limit=0.75)
        elif keycontroller.get("B"):
            trajectory_control = True
            
            scheduler.start()

        if trajectory_control and not scheduler.isFinished():
            scheduler.update()
            
            print("trajectory in progress", robot.position_targets[0])

        else:
            if trajectory_control:
                print("trajectory done")
                trajectory_control = False
            # robot.position_targets[active_joint] += 0.2 * stick.axes["LY"]
            

        # robot.printPositions()

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
