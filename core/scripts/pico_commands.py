#!/usr/bin/env python3

import serial
import struct
from time import sleep

def set_v(ser, v):
    data_to_send = struct.pack('f', v)
    data_to_send = b'\x03' + data_to_send
    ser.write(data_to_send)
	

if __name__ == '__main__':
    # ser = serial.Serial("/dev/ttyACM0", 115200)
    sleep(2)
    v = 1
    set_v(ser, v)
    
    #data_to_send = struct.pack('f', v)
    #data_to_send = b'\x03' + data_to_send
    #ser.write(data_to_send)
