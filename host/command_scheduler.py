import time

import numpy as np
from matplotlib import pyplot as plt


class Command:
    def __init__(self, duration=0):
        self.duration = duration
        self.t = 0
        self.is_finished = False
        self.is_running = False
        
    def update(self, t):
        if not self.is_finished:
            self.t = t
            if not self.is_running:
                self.initialize()
                self.is_running = True
            self.is_finished = self.execute()
            if self.is_finished:
                self.end()
                self.is_running = False
    
    def isRunning(self):
        return self.is_running
    
    def isFinished(self):
        return self.is_finished
    
    def initialize(self):
        pass
    
    def execute(self):
        return True
    
    def end(self):
        pass


class LogCommand(Command):
    def __init__(self, message):
        super().__init__()
        self.message = message
    
    def execute(self):
        print(self.message)
        return True


class WaitCommand(Command):
    def __init__(self, duration):
        super().__init__(duration)
    
    def execute(self):
        return self.t >= self.duration


class EaseInOutQuadTrajectoryCommand(Command):
    def __init__(self, robot, start_values, end_values, duration):
        super().__init__(duration)
        self.robot = robot
        self.start_values = start_values
        self.end_values = end_values
        self.target_values = start_values
        self.offset = 0
        
    def _easeInOutQuad(self, x):
        return 2 * x**2 if x < 0.5 else 1 - (-2 * x + 2)**2 / 2

    def execute(self):
        self.target_values = self.start_values + (self.end_values - self.start_values) * self._easeInOutQuad(self.t / self.duration)
        # print(self.t, self.target_value)
        if self.robot:
            self.robot.position_targets[0] = self.target_values[0]
            self.robot.position_targets[1] = self.target_values[1]
            self.robot.position_targets[2] = self.target_values[2]
            self.robot.position_targets[3] = self.target_values[3]
            self.robot.position_targets[4] = self.target_values[4]
            self.robot.position_targets[5] = self.target_values[5]
            self.robot.position_targets[6] = self.target_values[6]
            self.robot.position_targets[7] = self.target_values[7]
            self.robot.position_targets[8] = self.target_values[8]
            self.robot.position_targets[9] = self.target_values[9]
            self.robot.position_targets[10] = self.target_values[10]
            self.robot.position_targets[11] = self.target_values[11]

        return self.t >= self.duration


class Scheduler:
    def __init__(self):
        self.commands = []
        self.time_start = 0
        self.current_cmd_start_time = 0
    
    def addCommand(self, command):
        self.commands.append([self.current_cmd_start_time, command])
        self.current_cmd_start_time += command.duration
    
    def start(self):
        self.time_start = time.time()
    
    def update(self):
        for schedule_time, cmd in self.commands:
            t = time.time() - self.time_start
            if t >= schedule_time and not cmd.isFinished():
                cmd.update(t - schedule_time)
    
    def isFinished(self):
        for schedule_time, cmd in self.commands:
            if not cmd.isFinished():
                return False
        return True


if __name__ == "__main__":
    scheduler = Scheduler()

    scheduler.addCommand(LogCommand("hello"))
    scheduler.addCommand(WaitCommand(1))
    scheduler.addCommand(LogCommand("world"))
    scheduler.addCommand(EaseInOutQuadTrajectoryCommand(0, 10, 5))

    scheduler.start()

    while not scheduler.isFinished():
        scheduler.update()
        time.sleep(0.01)
