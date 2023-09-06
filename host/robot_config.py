import time

from robot import Humanoid
from keyboardcontroller import KeyboardController
import util


TRANSPORT = util.getTransport()

robot = Humanoid(transport=TRANSPORT)

configs = robot.getAllJointConfigs()

print(configs)
