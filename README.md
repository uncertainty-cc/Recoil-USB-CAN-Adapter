# Recoil USB-CAN Adapter

This adapter follows the python-can serial bus [API packet format](https://python-can.readthedocs.io/en/stable/interfaces/serial.html#serial-frame-format) to transmit CAN frames over UART.

## Recoil Motor Controller Communication Format

The CAN ID is separated into two fields, `device_id` and `func_id`. 

| func ID      | device ID    |
| ------------ | ------------ |
| 5 bits       | 6 bits       |
| CAN_ID[10:6] | CAN_ID[5:0]  |
| 32 functions | 64 devices   |


## Usage

```Python
COM_PORT = "COMx"  # COM port of the dongle

bus = can.Bus(interface="serial", channel=COM_PORT, baudrate=1000000)

```

## TODO

todo



## Setting up Pi CAN Hat

```bash
sudo apt update
sudo apt install python3-pip
```

```bash
pip3 install python-can
```

```bash
sudo nano /boot/config.txt
```

```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=1000000
```


```bash
reboot
```


```bash
dmesg | grep -i '\(can\|spi\)'
```
