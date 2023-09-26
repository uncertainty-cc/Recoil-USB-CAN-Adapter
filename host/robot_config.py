import json
import time

from robot import Humanoid
from keyboardcontroller import KeyboardController
import recoil
import util

TRANSPORT = util.getTransport()

robot = Humanoid(transport=TRANSPORT)
robot.start()

def dumpAllJointConfigs():
    configs = robot.getAllJointConfigs()

    json.dump(configs, open("./config/robot_config.json", "w"))


def loadAllJointConfigs():
    configs = json.load(open("./config/robot_config_v1.1_init.json", "r"))
    configs_prev = json.load(open("./config/robot_config_v1.0.json", "r"))


    for i, j in enumerate(robot.joints):
        print("")
        print("Configuring joint #{0}".format(i+1))
        config = configs_prev[str(i)]
        config_init = configs[str(i)]
        
        # print(config_init["positioncontroller"]["torque_limit"], config["positioncontroller"]["torque_limit"])
        # print(config_init["positioncontroller"]["position_kp"], config["positioncontroller"]["position_kp"])
        # print(config_init["positioncontroller"]["position_ki"], config["positioncontroller"]["position_ki"])
        # print(config_init["positioncontroller"]["velocity_kp"], config["positioncontroller"]["velocity_kp"])

        field = recoil.Command.ENCODER_FLUX_OFFSET
        val = config["encoder"]["flux_offset"]
        print(j._readParameterFloat(field)[1])
        j._writeParameterFloat(field, val)
        print(j._readParameterFloat(field)[1])
        j.storeSettingToFlash()

dumpAllJointConfigs()

# loadAllJointConfigs()

robot.stop()
