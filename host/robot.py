import time
import threading

import recoil

class Humanoid:
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
        
        self.torque_limit = 0.5

        self.target_positions = [0] * 12
        self.current_positions = [0] * 12

    def start(self):
        self.transport.start()
        self.is_stopped.clear()

    def stop(self):
        self.is_stopped.set()
        print("stopping...")

    def resetTargetPositions(self):
        for i, j in enumerate(self.joints):
            self.target_positions[i] = j.getPositionMeasured()
        self.printPositions()
        
    def printPositions(self):
        for i, j in enumerate(self.joints):
            print("{0}: {1:.3f}".format(j.device_id, self.target_positions[i]), end="\t")
        print()

    def setDamping(self):                
        for j in self.joints:
            j.setMode(recoil.Mode.DAMPING)
        
    def setIdle(self):
        for j in self.joints:
            j.setMode(recoil.Mode.IDLE)

    def rampUp(self):
        print("ramping up position control...")
        
        for i, j in enumerate(self.joints):
            j.setPositionTarget(self.target_positions[i])
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
                    j.setPositionTarget(self.target_positions[i])
                    j.setTorqueLimit(ramp_torque_limit)
                    j.feed()
                        
                print("{:.3f} / {:.3f}".format(ramp_torque_limit, self.torque_limit))

                time.sleep(dt)
            except KeyboardInterrupt:
                self.stop()
                continue
    
    def update(self):
        for i, j in enumerate(self.joints):
            j.setPositionTarget(self.target_positions[i])
            j.feed()
    
    def getJointConfigs(self, joint_id):
        config = {}

        config["id"]                                    = self.joints[joint_id].getInfo()[0]
        config["version"]                               = self.joints[joint_id].getInfo()[1]
        config["mode"]                                  = self.joints[joint_id].getMode()

        config["encoder"] = {}
        config["encoder"]["cpr"]                        = self.joints[joint_id]._readParameterInt32(recoil.Command.ENCODER_CPR)[1]
        config["encoder"]["position_offset"]            = self.joints[joint_id]._readParameterFloat(recoil.Command.ENCODER_OFFSET)[1]
        config["encoder"]["filter_bandwidth"]           = self.joints[joint_id]._readParameterFloat(recoil.Command.ENCODER_FILTER_BANDWIDTH)[1]
        config["encoder"]["flux_offset"]                = self.joints[joint_id]._readParameterFloat(recoil.Command.ENCODER_FLUX_OFFSET)[1]
        config["encoder"]["position_raw"]               = self.joints[joint_id]._readParameterInt32(recoil.Command.ENCODER_POSITION_RAW)[1]
        config["encoder"]["n_rotations"]                = self.joints[joint_id]._readParameterInt32(recoil.Command.ENCODER_N_ROTATIONS)[1]

        config["powerstage"] = {}
        config["powerstage"]["undervoltage_threshold"]  = self.joints[joint_id]._readParameterFloat(recoil.Command.POWERSTAGE_VOLTAGE_THRESHOLD_LOW)[1]
        config["powerstage"]["overvoltage_threshold"]   = self.joints[joint_id]._readParameterFloat(recoil.Command.POWERSTAGE_VOLTAGE_THRESHOLD_HIGH)[1]
        config["powerstage"]["bus_voltage_filter_alpha"] = self.joints[joint_id]._readParameterFloat(recoil.Command.POWERSTAGE_FILTER)[1]
        config["powerstage"]["bus_voltage_measured"]    = self.joints[joint_id]._readParameterFloat(recoil.Command.POWERSTAGE_BUS_VOLTAGE_MEASURED)[1]

        config["motor"] = {}
        config["motor"]["pole_pairs"]                   = self.joints[joint_id]._readParameterUInt32(recoil.Command.MOTOR_POLE_PAIR)[1]
        config["motor"]["kv_rating"]                    = self.joints[joint_id]._readParameterUInt32(recoil.Command.MOTOR_KV)[1]
        config["motor"]["phase_order"]                  = self.joints[joint_id]._readParameterInt32(recoil.Command.MOTOR_PHASE_ORDER)[1]
        config["motor"]["phase_resistance"]             = self.joints[joint_id]._readParameterFloat(recoil.Command.MOTOR_PHASE_RESISTANCE)[1]
        config["motor"]["phase_inductance"]             = self.joints[joint_id]._readParameterFloat(recoil.Command.MOTOR_PHASE_INDUCTANCE)[1]
        # config["motor"]["max_calibration_current"]      = self.joints[joint_id]._readParameterFloat(recoil.Command.MOTOR_MAX_CALIBRATION_CURRENT)[1]

        config["currentcontroller"] = {}
        config["currentcontroller"]["i_kp"]             = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_KP)[1]
        config["currentcontroller"]["i_ki"]             = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_KI)[1]
        config["currentcontroller"]["i_bandwidth"]      = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_BANDWIDTH)[1]
        config["currentcontroller"]["i_limit"]          = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_LIMIT)[1]
        config["currentcontroller"]["i_a_measured"]     = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_IA_MEASURED)[1]
        config["currentcontroller"]["i_b_measured"]     = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_IB_MEASURED)[1]
        config["currentcontroller"]["i_c_measured"]     = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_IC_MEASURED)[1]
        config["currentcontroller"]["v_a_setpoint"]     = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_VA_SETPOINT)[1]
        config["currentcontroller"]["v_b_setpoint"]     = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_VB_SETPOINT)[1]
        config["currentcontroller"]["v_c_setpoint"]     = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_VC_SETPOINT)[1]
        config["currentcontroller"]["i_alpha_measured"] = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_IALPHA_MEASURED)[1]
        config["currentcontroller"]["i_beta_measured"]  = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_IBETA_MEASURED)[1]
        config["currentcontroller"]["v_alpha_setpoint"] = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_VALPHA_SETPOINT)[1]
        config["currentcontroller"]["v_beta_setpoint"]  = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_VBETA_SETPOINT)[1]
        config["currentcontroller"]["v_q_target"]       = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_VQ_TARGET)[1]
        config["currentcontroller"]["v_d_target"]       = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_VD_TARGET)[1]
        config["currentcontroller"]["v_q_setpoint"]     = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_VQ_SETPOINT)[1]
        config["currentcontroller"]["v_d_setpoint"]     = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_VD_SETPOINT)[1]
        config["currentcontroller"]["i_q_target"]       = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_IQ_TARGET)[1]
        config["currentcontroller"]["i_d_target"]       = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_ID_TARGET)[1]
        config["currentcontroller"]["i_q_measured"]     = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_IQ_MEASURED)[1]
        config["currentcontroller"]["i_d_measured"]     = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_ID_MEASURED)[1]
        config["currentcontroller"]["i_q_setpoint"]     = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_IQ_SETPOINT)[1]
        config["currentcontroller"]["i_d_setpoint"]     = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_ID_SETPOINT)[1]
        config["currentcontroller"]["i_q_integrator"]   = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_IQ_INTEGRATOR)[1]
        config["currentcontroller"]["i_d_integrator"]   = self.joints[joint_id]._readParameterFloat(recoil.Command.CURRENT_ID_INTEGRATOR)[1]

        config["positioncontroller"] = {}
        config["positioncontroller"]["position_kp"]             = self.joints[joint_id]._readParameterFloat(recoil.Command.POSITION_KP)[1]
        config["positioncontroller"]["position_ki"]             = self.joints[joint_id]._readParameterFloat(recoil.Command.POSITION_KI)[1]
        config["positioncontroller"]["velocity_kp"]             = self.joints[joint_id]._readParameterFloat(recoil.Command.VELOCITY_KP)[1]
        config["positioncontroller"]["velocity_ki"]             = self.joints[joint_id]._readParameterFloat(recoil.Command.VELOCITY_KI)[1]
        config["positioncontroller"]["torque_limit"]            = self.joints[joint_id]._readParameterFloat(recoil.Command.TORQUE_LIMIT)[1]
        config["positioncontroller"]["velocity_limit"]          = self.joints[joint_id]._readParameterFloat(recoil.Command.VELOCITY_LIMIT)[1]
        config["positioncontroller"]["position_limit_lower"]    = self.joints[joint_id]._readParameterFloat(recoil.Command.POSITION_LIMIT_LOW)[1]
        config["positioncontroller"]["position_limit_upper"]    = self.joints[joint_id]._readParameterFloat(recoil.Command.POSITION_LIMIT_HIGH)[1]
        config["positioncontroller"]["torque_target"]           = self.joints[joint_id]._readParameterFloat(recoil.Command.TORQUE_TARGET)[1]
        config["positioncontroller"]["torque_measured"]         = self.joints[joint_id]._readParameterFloat(recoil.Command.TORQUE_MEASURED)[1]
        config["positioncontroller"]["torque_setpoint"]         = self.joints[joint_id]._readParameterFloat(recoil.Command.TORQUE_SETPOINT)[1]
        config["positioncontroller"]["velocity_target"]         = self.joints[joint_id]._readParameterFloat(recoil.Command.VELOCITY_TARGET)[1]
        config["positioncontroller"]["velocity_measured"]       = self.joints[joint_id]._readParameterFloat(recoil.Command.VELOCITY_MEASURED)[1]
        config["positioncontroller"]["velocity_setpoint"]       = self.joints[joint_id]._readParameterFloat(recoil.Command.VELOCITY_SETPOINT)[1]
        config["positioncontroller"]["position_target"]         = self.joints[joint_id]._readParameterFloat(recoil.Command.POSITION_TARGET)[1]
        config["positioncontroller"]["position_measured"]       = self.joints[joint_id]._readParameterFloat(recoil.Command.POSITION_MEASURED)[1]
        config["positioncontroller"]["position_setpoint"]       = self.joints[joint_id]._readParameterFloat(recoil.Command.POSITION_SETPOINT)[1]
        config["positioncontroller"]["velocity_integrator"]     = self.joints[joint_id]._readParameterFloat(recoil.Command.VELOCITY_INTEGRATOR)[1]
        config["positioncontroller"]["position_integrator"]     = self.joints[joint_id]._readParameterFloat(recoil.Command.POSITION_INTEGRATOR)[1]

        return config

    def getAllJointConfigs(self):
        config = {}
        for i, j in enumerate(self.joints):
            config[i] = self.getJointConfigs(i)
        return config
    