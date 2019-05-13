#!/usr/bin/python
import serial
import time
import sys
from binascii import hexlify

COMPORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
ser = serial.Serial(COMPORT, 76800)

def m365_send(length, addr, mode, offset, data):
    for i in range(5):
        ser.write(0xFF) # Wake
        time.sleep(0.01)

    arg = [length, addr, mode, offset]
    arg.extend(data)
    crc = sum(arg) ^ 0xFFFF

    send = [0x55, 0xAA]
    send.extend(arg)

    send.append(crc & 0xFF)
    send.append((crc >> 8) & 0xFF)
    send = bytes(send)
    print(hexlify(send))
    ser.write(send)

# Reboots to bootloader
def bootloader():
    m365_send(3, 0x22, 0xFA, 11, [0])

bootloader()
