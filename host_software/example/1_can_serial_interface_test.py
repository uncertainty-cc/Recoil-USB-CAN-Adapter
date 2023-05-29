import time

import can
import can.interfaces.serial
import serial


def sendOneMessage():
    """Sends a single message."""

    counter = 0
    
    # bus = can.Bus(interface="serial", channel="COM10", baudrate=1000000)
    bus = can.Bus(interface="serial", channel="COM14", baudrate=115200)

    msg = can.Message(
        arbitration_id=0x01,
        data=[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
        is_extended_id=False
    )

    try:
        bus.send(msg)
        print(f"Message sent on {bus.channel_info}")
    except can.CanError as e:
        print("Message NOT sent:", e)
    except serial.serialutil.SerialException as e:
        print("Message NOT sent:", e)

def sendManyMessage():
    while True:
        sendOneMessage()
        

if __name__ == "__main__":
    sendOneMessage()
    sendManyMessage()
