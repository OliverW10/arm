#!/usr/bin/python3

import time
import serial
import glob
from util import clamp
import ik

def sendAngles(ser, yaw, pitch, wrist):
    yaw = clamp(yaw + ik.offset_1, 0, 180)
    pitch = clamp(pitch + ik.offset_2, 0, 180)
    wrist = clamp(wrist + ik.offset_3, 0, 180)
    string = ",".join(str(round(x)) for x in [yaw, pitch, wrist])+";"
    ser.write(bytes(string, "utf-8"))

serial_devices = glob.glob("/dev/ttyACM*")
print(serial_devices)
if len(serial_devices) == 0:
    print("arduino not found")
ser = serial.Serial(serial_devices[0])
time.sleep(1)
sendAngles(ser, 90, 90, 90)
time.sleep(3)
ser.close()
