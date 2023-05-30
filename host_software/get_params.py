import time

import can.interfaces.serial

import recoil

DEVICE_ID = 1

F_LEN = 24

transport = recoil.SerialCANTransport(port="/dev/ttyACM0", baudrate=115200)
transport.start()

motor = recoil.MotorController(transport, device_id=DEVICE_ID)

config = {}
config["encoder"] = {}
config["encoder"]["cpr"] =                      motor._readParameterInt32(recoil.Command.ENCODER_CPR)[1]
config["encoder"]["position_offset"] =          motor._readParameterFloat(recoil.Command.ENCODER_OFFSET)[1]
config["encoder"]["filter_alpha"] =             motor._readParameterFloat(recoil.Command.ENCODER_FILTER)[1]
config["encoder"]["flux_offset"] =              motor._readParameterFloat(recoil.Command.ENCODER_FLUX_OFFSET)[1]
config["encoder"]["position_raw"] =             motor._readParameterInt32(recoil.Command.ENCODER_POSITION_RAW)[1]
config["encoder"]["n_rotations"] =              motor._readParameterInt32(recoil.Command.ENCODER_N_ROTATIONS)[1]

config["powerstage"] = {}
config["powerstage"]["undervoltage_threshold"] =   motor._readParameterFloat(recoil.Command.POWERSTAGE_VOLTAGE_THRESHOLD_LOW)[1]
config["powerstage"]["overvoltage_threshold"] =    motor._readParameterFloat(recoil.Command.POWERSTAGE_VOLTAGE_THRESHOLD_HIGH)[1]
config["powerstage"]["bus_voltage_filter_alpha"] = motor._readParameterFloat(recoil.Command.POWERSTAGE_FILTER)[1]
config["powerstage"]["bus_voltage_measured"] =     motor._readParameterFloat(recoil.Command.POWERSTAGE_BUS_VOLTAGE_MEASURED)[1]

config["motor"] = {}
config["motor"]["pole_pairs"] =               motor._readParameterUInt32(recoil.Command.MOTOR_POLE_PAIR)[1]
config["motor"]["kv_rating"] =                motor._readParameterUInt32(recoil.Command.MOTOR_KV)[1]
config["motor"]["phase_order"] =              motor._readParameterInt32(recoil.Command.MOTOR_PHASE_ORDER)[1]
config["motor"]["phase_resistance"] =         motor._readParameterFloat(recoil.Command.MOTOR_PHASE_RESISTANCE)[1]
config["motor"]["phase_inductance"] =         motor._readParameterFloat(recoil.Command.MOTOR_PHASE_INDUCTANCE)[1]

config["currentcontroller"] = {}
config["currentcontroller"]["i_kp"] =                     motor._readParameterFloat(recoil.Command.CURRENT_KP)[1]
config["currentcontroller"]["i_ki"] =                     motor._readParameterFloat(recoil.Command.CURRENT_KI)[1]
config["currentcontroller"]["i_bandwidth"] =              motor._readParameterFloat(recoil.Command.CURRENT_BANDWIDTH)[1]
config["currentcontroller"]["i_limit"] =                  motor._readParameterFloat(recoil.Command.CURRENT_LIMIT)[1]
config["currentcontroller"]["i_a_measured"] =             motor._readParameterFloat(recoil.Command.CURRENT_IA_MEASURED)[1]
config["currentcontroller"]["i_b_measured"] =             motor._readParameterFloat(recoil.Command.CURRENT_IB_MEASURED)[1]
config["currentcontroller"]["i_c_measured"] =             motor._readParameterFloat(recoil.Command.CURRENT_IC_MEASURED)[1]
config["currentcontroller"]["v_a_setpoint"] =             motor._readParameterFloat(recoil.Command.CURRENT_VA_SETPOINT)[1]
config["currentcontroller"]["v_b_setpoint"] =             motor._readParameterFloat(recoil.Command.CURRENT_VB_SETPOINT)[1]
config["currentcontroller"]["v_c_setpoint"] =             motor._readParameterFloat(recoil.Command.CURRENT_VC_SETPOINT)[1]
config["currentcontroller"]["i_alpha_measured"] =         motor._readParameterFloat(recoil.Command.CURRENT_IALPHA_MEASURED)[1]
config["currentcontroller"]["i_beta_measured"] =          motor._readParameterFloat(recoil.Command.CURRENT_IBETA_MEASURED)[1]
config["currentcontroller"]["v_alpha_setpoint"] =         motor._readParameterFloat(recoil.Command.CURRENT_VALPHA_SETPOINT)[1]
config["currentcontroller"]["v_beta_setpoint"] =          motor._readParameterFloat(recoil.Command.CURRENT_VBETA_SETPOINT)[1]
config["currentcontroller"]["v_q_target"] =               motor._readParameterFloat(recoil.Command.CURRENT_VQ_TARGET)[1]
config["currentcontroller"]["v_d_target"] =               motor._readParameterFloat(recoil.Command.CURRENT_VD_TARGET)[1]
config["currentcontroller"]["v_q_setpoint"] =             motor._readParameterFloat(recoil.Command.CURRENT_VQ_SETPOINT)[1]
config["currentcontroller"]["v_d_setpoint"] =             motor._readParameterFloat(recoil.Command.CURRENT_VD_SETPOINT)[1]
config["currentcontroller"]["i_q_target"] =               motor._readParameterFloat(recoil.Command.CURRENT_IQ_TARGET)[1]
config["currentcontroller"]["i_d_target"] =               motor._readParameterFloat(recoil.Command.CURRENT_ID_TARGET)[1]
config["currentcontroller"]["i_q_measured"] =             motor._readParameterFloat(recoil.Command.CURRENT_IQ_MEASURED)[1]
config["currentcontroller"]["i_d_measured"] =             motor._readParameterFloat(recoil.Command.CURRENT_ID_MEASURED)[1]
config["currentcontroller"]["i_q_setpoint"] =             motor._readParameterFloat(recoil.Command.CURRENT_IQ_SETPOINT)[1]
config["currentcontroller"]["i_d_setpoint"] =             motor._readParameterFloat(recoil.Command.CURRENT_ID_SETPOINT)[1]
config["currentcontroller"]["i_q_integrator"] =           motor._readParameterFloat(recoil.Command.CURRENT_IQ_INTEGRATOR)[1]
config["currentcontroller"]["i_d_integrator"] =           motor._readParameterFloat(recoil.Command.CURRENT_ID_INTEGRATOR)[1]

config["positioncontroller"] = {}
config["positioncontroller"]["position_kp"] =              motor._readParameterFloat(recoil.Command.POSITION_KP)[1]
config["positioncontroller"]["position_ki"] =              motor._readParameterFloat(recoil.Command.POSITION_KI)[1]
config["positioncontroller"]["velocity_kp"] =              motor._readParameterFloat(recoil.Command.VELOCITY_KP)[1]
config["positioncontroller"]["velocity_ki"] =              motor._readParameterFloat(recoil.Command.VELOCITY_KI)[1]
config["positioncontroller"]["torque_limit"] =             motor._readParameterFloat(recoil.Command.TORQUE_LIMIT)[1]
config["positioncontroller"]["velocity_limit"] =           motor._readParameterFloat(recoil.Command.VELOCITY_LIMIT)[1]
config["positioncontroller"]["position_limit_lower"] =     motor._readParameterFloat(recoil.Command.POSITION_LIMIT_LOW)[1]
config["positioncontroller"]["position_limit_upper"] =     motor._readParameterFloat(recoil.Command.POSITION_LIMIT_HIGH)[1]
config["positioncontroller"]["torque_target"] =            motor._readParameterFloat(recoil.Command.TORQUE_TARGET)[1]
config["positioncontroller"]["torque_measured"] =          motor._readParameterFloat(recoil.Command.TORQUE_MEASURED)[1]
config["positioncontroller"]["torque_setpoint"] =          motor._readParameterFloat(recoil.Command.TORQUE_SETPOINT)[1]
config["positioncontroller"]["velocity_target"] =          motor._readParameterFloat(recoil.Command.VELOCITY_TARGET)[1]
config["positioncontroller"]["velocity_measured"] =        motor._readParameterFloat(recoil.Command.VELOCITY_MEASURED)[1]
config["positioncontroller"]["velocity_setpoint"] =        motor._readParameterFloat(recoil.Command.VELOCITY_SETPOINT)[1]
config["positioncontroller"]["position_target"] =          motor._readParameterFloat(recoil.Command.POSITION_TARGET)[1]
config["positioncontroller"]["position_measured"] =        motor._readParameterFloat(recoil.Command.POSITION_MEASURED)[1]
config["positioncontroller"]["position_setpoint"] =        motor._readParameterFloat(recoil.Command.POSITION_SETPOINT)[1]
config["positioncontroller"]["velocity_integrator"] =      motor._readParameterFloat(recoil.Command.VELOCITY_INTEGRATOR)[1]
config["positioncontroller"]["position_integrator"] =      motor._readParameterFloat(recoil.Command.POSITION_INTEGRATOR)[1]

print("## Encoder Settings")
for key in config["encoder"].keys():
    print(" - {0}:".format(key).ljust(F_LEN, " "), config["encoder"][key])

print("## PowerStage Settings")
for key in config["powerstage"].keys():
    print(" - {0}:".format(key).ljust(F_LEN, " "), config["powerstage"][key])

print("## Motor Settings")
for key in config["motor"].keys():
    print(" - {0}:".format(key).ljust(F_LEN, " "), config["motor"][key])

print("## CurrentController Settings")
for key in config["currentcontroller"].keys():
    print(" - {0}:".format(key).ljust(F_LEN, " "), config["currentcontroller"][key])

print("## PositionController Settings")
for key in config["positioncontroller"].keys():
    print(" - {0}:".format(key).ljust(F_LEN, " "), config["positioncontroller"][key])
