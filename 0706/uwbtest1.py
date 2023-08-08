import time
import numpy as np # ver 1.19.4
from imutils.video import VideoStream
import argparse
import imutils
import cv2 # ver 3.4.18.65
import sys
import json
#uwb
import serial
import struct
from threading import Thread

ser = serial.Serial('/dev/ttyTHS1',115200,timeout=1)
uwb1 = 0
uwb2 = 0

def UWB():
    global uwb1,uwb2
    if ser.in_waiting >= 5:
        id = ser.read()
        if id == b'\x14':
            data1 = ser.read(2)
            data2 = ser.read(2)
            values1 = struct.unpack('<H',data1)[0]
            values2 = struct.unpack('<H',data2)[0]-50
            uwb1 = values1
            uwb2 = values2
#    return uwb1,uwb2

while True:
    U = Thread(target=UWB)
    U.start()
    U.join()
    if(uwb1 == 0)and(uwb2 == 0):   
        print("pass")
        continue
    print("dist1:{},dist2:{}".format(uwb1,uwb2))


