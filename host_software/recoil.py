import time
import os
import struct
import threading

import can
import serial

class Mode:
    # these are three safe modes
    DISABLED                        = 0x00
    IDLE                            = 0x01

    # these are special modes
    DAMPING                         = 0x02
    CALIBRATION                     = 0x05

    # these are closed-loop modes
    CURRENT                         = 0x10
    TORQUE                          = 0x11
    VELOCITY                        = 0x12
    POSITION                        = 0x13

    # these are open-loop modes
    VABC_OVERRIDE                   = 0x20
    VALPHABETA_OVERRIDE             = 0x21
    VQD_OVERRIDE                    = 0x22

    DEBUG                           = 0x80

class ErrorCode:
    NO_ERROR                        = 0b0000000000000000
    GENERAL                         = 0b0000000000000001
    ESTOP                           = 0b0000000000000010
    INITIALIZATION_ERROR            = 0b0000000000000100
    CALIBRATION_ERROR               = 0b0000000000001000
    POWERSTAGE_ERROR                = 0b0000000000010000
    INVALID_MODE                    = 0b0000000000100000
    WATCHDOG_TIMEOUT                = 0b0000000001000000
    OVER_VOLTAGE                    = 0b0000000010000000
    OVER_CURRENT                    = 0b0000000100000000
    OVER_TEMPERATURE                = 0b0000001000000000
    CAN_TX_FAULT                    = 0b0000010000000000
    I2C_FAULT                       = 0b0000100000000000

class CAN_ID:
    ESTOP                           = 0x00
    INFO                            = 0x01
    SAFETY_WATCHDOG                 = 0x02
    MODE                            = 0x05
    FLASH                           = 0x0E

    USR_PARAM_READ                  = 0x10
    USR_PARAM_WRITE                 = 0x11
    USR_FAST_FRAME_0                = 0x12
    USR_FAST_FRAME_1                = 0x13
    USR_DEBUG_0                     = 0x14
    USR_DEBUG_1                     = 0x15
    USR_DEBUG_2                     = 0x16

    PING                            = 0x1F

class Command:
    ENCODER_CPR                     = 0x10
    ENCODER_OFFSET                  = 0x11
    ENCODER_FILTER                  = 0x12
    ENCODER_FLUX_OFFSET             = 0x13
    ENCODER_POSITION_RAW            = 0x14
    ENCODER_N_ROTATIONS             = 0x15
    POWERSTAGE_VOLTAGE_THRESHOLD_LOW    = 0x16
    POWERSTAGE_VOLTAGE_THRESHOLD_HIGH   = 0x17
    POWERSTAGE_FILTER                   = 0x18
    POWERSTAGE_BUS_VOLTAGE_MEASURED     = 0x19
    MOTOR_POLE_PAIR                 = 0x1A
    MOTOR_KV                        = 0x1B
    MOTOR_PHASE_ORDER               = 0x1C
    MOTOR_PHASE_RESISTANCE          = 0x1D
    MOTOR_PHASE_INDUCTANCE          = 0x1E
    CURRENT_KP                      = 0x1F
    CURRENT_KI                      = 0x20
    CURRENT_BANDWIDTH               = 0x21
    CURRENT_LIMIT                   = 0x22
    CURRENT_IA_MEASURED             = 0x23
    CURRENT_IB_MEASURED             = 0x24
    CURRENT_IC_MEASURED             = 0x25
    CURRENT_VA_SETPOINT             = 0x26
    CURRENT_VB_SETPOINT             = 0x27
    CURRENT_VC_SETPOINT             = 0x28
    CURRENT_IALPHA_MEASURED         = 0x29
    CURRENT_IBETA_MEASURED          = 0x2A
    CURRENT_VALPHA_SETPOINT         = 0x2B
    CURRENT_VBETA_SETPOINT          = 0x2C
    CURRENT_VQ_TARGET               = 0x2D
    CURRENT_VD_TARGET               = 0x2E
    CURRENT_VQ_SETPOINT             = 0x2F
    CURRENT_VD_SETPOINT             = 0x30
    CURRENT_IQ_TARGET               = 0x31
    CURRENT_ID_TARGET               = 0x32
    CURRENT_IQ_MEASURED             = 0x33
    CURRENT_ID_MEASURED             = 0x34
    CURRENT_IQ_SETPOINT             = 0x35
    CURRENT_ID_SETPOINT             = 0x36
    CURRENT_IQ_INTEGRATOR           = 0x37
    CURRENT_ID_INTEGRATOR           = 0x38
    POSITION_KP                     = 0x39
    POSITION_KI                     = 0x3A
    VELOCITY_KP                     = 0x3B
    VELOCITY_KI                     = 0x3C
    TORQUE_LIMIT                    = 0x3D
    VELOCITY_LIMIT                  = 0x3E
    POSITION_LIMIT_LOW              = 0x3F
    POSITION_LIMIT_HIGH             = 0x40
    TORQUE_TARGET                   = 0x41
    TORQUE_MEASURED                 = 0x42
    TORQUE_SETPOINT                 = 0x43
    VELOCITY_TARGET                 = 0x44
    VELOCITY_MEASURED               = 0x45
    VELOCITY_SETPOINT               = 0x46
    POSITION_TARGET                 = 0x47
    POSITION_MEASURED               = 0x48
    POSITION_SETPOINT               = 0x49
    VELOCITY_INTEGRATOR             = 0x4A
    POSITION_INTEGRATOR             = 0x4B



class CANFrame:
    ID_STANDARD = 0
    ID_EXTENDED = 1
    
    def __init__(self, 
            device_id=0, 
            func_id=0, 
            size=0, 
            data=b"",
            id_type=ID_STANDARD
        ):
        self.device_id = device_id
        self.func_id = func_id
        self.size = size
        self.data = data
        self.id_type = id_type


class SPICANTransport:
    def __init__(self, port="can0", baudrate=1000000):
        self.port = port
        self.baudrate = baudrate
        self._interface = None
        self._handlers = []
        self._killed = threading.Event()

    def stop(self):
        self._killed.set()
        os.system("sudo ifconfig {port} down".format(port=self.port))

    def start(self):
        os.system("sudo ip link set {port} type can bitrate {baudrate}".format(port=self.port, baudrate=self.baudrate))
        os.system("sudo ifconfig {port} up".format(port=self.port))

        self._killed.clear()
        self.connect()

        print("started")

    def connect(self):
        while not self._interface:
            try:
                self._interface = can.Bus(interface="socketcan", channel=self.port, bustype="socketcan", baudrate=self.baudrate)
            except serial.serialutil.SerialException as e:
                print(e)
        print("connected")

    def transmit(self, controller, frame, callback=None):
        can_id = (frame.func_id << 6) | frame.device_id

        msg = can.Message(
            arbitration_id=can_id,
            is_extended_id=False,
            data=frame.data)
        self._interface.send(msg)
            
    def receive(self, controller, timeout=0.1):
        try:
            msg = self._interface.recv(timeout=timeout) # blocking
        except can.exceptions.CanOperationError as e:
            print(e)
            return None
        while msg and msg.is_error_frame:
            try:
                msg = self._interface.recv(timeout=timeout) # blocking
            except can.exceptions.CanOperationError as e:
                print(e)
                return None
        # print(msg)
        if not msg:
            return None
        frame = CANFrame(
            device_id = msg.arbitration_id & 0x3F,
            func_id = msg.arbitration_id >> 6,
            size = msg.dlc,
            data = msg.data
        )
        return frame


class SerialCANTransport:
    def __init__(self, port="COM1", baudrate=1000000):
        self.port = port
        self.baudrate = baudrate
        self._interface = None
        self._handlers = []
        self._killed = threading.Event()

    def stop(self):
        self._killed.set()

    def start(self):
        self._killed.clear()
        self.connect()

        print("started")

    def connect(self):
        while not self._interface:
            try:
                self._interface = can.Bus(interface="serial", channel=self.port, baudrate=self.baudrate)
            except serial.serialutil.SerialException as e:
                print(e)
        print("connected")

    def transmit(self, controller, frame):
        can_id = (frame.func_id << 6) | frame.device_id

        msg = can.Message(
            arbitration_id=can_id,
            is_extended_id=False,
            data=frame.data)
        self._interface.send(msg)
        
    def receive(self, controller, timeout=0.1):
        try:
            msg = self._interface.recv(timeout=timeout) # blocking
        except can.exceptions.CanOperationError:
            return None
        if not msg:
            return None
        frame = CANFrame(
            device_id = msg.arbitration_id & 0x3F,
            func_id = msg.arbitration_id >> 6,
            size = msg.dlc,
            data = msg.data
        )
        return frame


class MotorController:
    def __init__(self, transport, device_id=1):
        self.transport = transport
        self.device_id = device_id
        
        self.mode = Mode.DISABLED
        self.firmware_version = ""

    @staticmethod
    def unpack(format_str, data, index=0):
        try:
            return struct.unpack(format_str, data)[index]
        except struct.error as e:
            print("warning:", e, data)
            return 0

    def ping(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.PING, size=0)
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<B", rx_frame.data, 0)
        if callback:
            callback(self, rx_data)
        return rx_data

    def feed(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.SAFETY_WATCHDOG, size=0)
        self.transport.transmit(self, tx_frame)

    def getMode(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.MODE_ERROR, size=0)
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<HH", rx_frame.data, 0)
        if callback:
            callback(self, rx_data)
        return rx_data

    def setMode(self, mode, callback=None, clear_error=False):
        tx_frame = CANFrame(self.device_id, CAN_ID.MODE_ERROR, size=4, 
                            data=struct.pack("<HH", mode, 1 if clear_error else 0))
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<HH", rx_frame.data, 0)
        if callback:
            callback(self, rx_data)
        return rx_data

    def loadSettingFromFlash(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.FLASH, size=1, 
                            data=struct.pack("<B", 0))
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<B", rx_frame.data, 0)
        if callback:
            callback(rx_data)
        return rx_data
        
    def storeSettingToFlash(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.FLASH, size=1, 
                            data=struct.pack("<B", 1))
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<B", rx_frame.data, 0)
        if callback:
            callback(rx_data)
        return rx_data

    def readParameter(self, param_cmd, callback=None):
        tx_data = struct.pack("<B", param_cmd)
        tx_frame = CANFrame(self.device_id, CAN_ID.USR_PARAM_READ, size=1, data=tx_data)
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_cmd = MotorController.unpack("<BBBBf", rx_frame.data)[0]
        rx_data = MotorController.unpack("<BBBBf", rx_frame.data)[4]
        print(rx_cmd, rx_data)
        if callback:
            callback(rx_data)
        return rx_data

    def writeParameter(self, param_cmd, value, callback=None):
        tx_data = None
        if type(value) == float:
            tx_data = struct.pack("<BBBBf", param_cmd, 0, 0, 0, value)
        elif type(value) == int:
            tx_data = struct.pack("<BBBBL", param_cmd, 0, 0, 0, value)
        else:
            print("unspported type!")
        tx_frame = CANFrame(self.device_id, CAN_ID.USR_PARAM_WRITE, size=8, data=tx_data)
        self.transport.transmit(self, tx_frame)
    
    def getPositionMeasured(self):
        return self.readParameter(Command.POSITION_MEASURED)