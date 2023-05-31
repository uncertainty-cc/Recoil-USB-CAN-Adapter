import time
import threading

import can
import can.interfaces.serial
import serial


def nodeA(port, is_stopped):
    bus = can.Bus(interface="serial", channel=port, baudrate=1000000)
    while not is_stopped.is_set():
        msg = can.Message(
            arbitration_id=0x01,
            data=[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
            is_extended_id=False
        )

        try:
            bus.send(msg)
            #print(f"Message sent on {bus.channel_info}")
        except can.CanError as e:
            print("Message NOT sent:", e)
        except serial.serialutil.SerialException as e:
            print("Message NOT sent:", e)

        try:
            msg = bus.recv(timeout=0.01) # blocking
        except can.exceptions.CanOperationError:
            continue
        # print(msg)

def nodeB(port, is_stopped):
    bus = can.Bus(interface="serial", channel=port, baudrate=1000000)
    while not is_stopped.is_set():
        
        try:
            msg = bus.recv(timeout=0.01) # blocking
        except can.exceptions.CanOperationError:
            continue
        # print(msg)
        
        msg = can.Message(
            arbitration_id=0x02,
            data=[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
            is_extended_id=False
        )

        try:
            bus.send(msg)
            #print(f"Message sent on {bus.channel_info}")
        except can.CanError as e:
            print("Message NOT sent:", e)
        except serial.serialutil.SerialException as e:
            print("Message NOT sent:", e)


        

if __name__ == "__main__":
    is_stopped = threading.Event()

    print("started")
    try:
        nodeB("COM10", is_stopped)
    except KeyboardInterrupt:
        is_stopped.set()
        print("Thread stopped")

