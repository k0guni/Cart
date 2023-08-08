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
import signal
# import matplotlib
#import matplotlib.pyplot as plt

line1 = []
line2 = []
uwb1 = 0
uwb2 = 0
num = 0

exitThread = False

def handler(signum, frame):
    global exitThread
    exitThread = True

def parsing_data(data):
    tmp = ''.join(data)

    print(tmp)

def readThread(ser):
    global line1
    global line2
    global exitThread

    while not exitThread:
        if ser.in_waiting >= 5:
            data1 = ser.read(2)
            data2 = ser.read(2)
            values1 = struct.unpack('<H',data1)[0]
            values2 = struct.unpack('<H',data2)[0]-50
            print("{},{}".format(values1,values2))
            # line1.append((values1))
            # line2.append((values2))
            # parsing_data(line1)
            # parsing_data(line2)

            #del line1[:], line2[:]

def UWB():
    global uwb1,uwb2
    if ser.in_waiting >= 5:
        data1 = ser.read(2)
        data2 = ser.read(2)
        values1 = struct.unpack('<H',data1)[0]
        values2 = struct.unpack('<H',data2)[0]-50
        #if (values1 < 3000)and(values1 != 2580)and(values1 != 1300)and(values1 != 2068)and(values2 < 3000)and(values2 != 2580)and(values2 != 1300)and(values2 != 2068)and(values2 != 1812):
        uwb1 = values1
        uwb2 = values2

#plt.figure()
while True:
    signal.signal(signal.SIGINT, handler)
    ser = serial.Serial('/dev/ttyTHS1',115200,timeout=1)
    thread = Thread(target=readThread, args=(ser,))
    thread.start()
    # time.sleep(0.1)
    # plt.plot(num,uwb1,'r.-',num,uwb2,'b.-')        
    # plt.pause(0.01)


