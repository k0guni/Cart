import time
import numpy as np
from imutils.video import VideoStream
import argparse
import imutils
import cv2
import sys
import json
import serial
import struct
from threading import Thread
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)
num = 0
uwb1_values = []
uwb2_values = []

def UWB():
    if ser.in_waiting >= 5:
        id = ser.read()
        if id == b'\x14':
            data1 = ser.read(2)
            data2 = ser.read(2)
            values1 = struct.unpack('<H', data1)[0]
            values2 = struct.unpack('<H', data2)[0] - 50
            return values1, values2
    return None, None

plt.figure()
while True:
    uwb1, uwb2 = UWB()
    if uwb1 is not None and uwb2 is not None:
        num += 1
        print("dist1: {}, dist2: {}".format(uwb1, uwb2))
        uwb1_values.append(uwb1)
        uwb2_values.append(uwb2)
        plt.plot(num, uwb1, 'rs--', num, uwb2, 'bs--')
        plt.pause(0.01)
