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
# import matplotlib
#import matplotlib.pyplot as plt

ser = serial.Serial('/dev/ttyTHS1',115200,timeout=1)
uwblist = []
uwb1 = 0
uwb2 = 0
num = 0
nummain = 0
def UWB():
    global uwb1,uwb2,num,uwblist
    while True:
        if ser.in_waiting >= 5:
            id = ser.read()
            if id == b'\x14':
                num += 1
                data1 = ser.read(2)
                data2 = ser.read(2)
                values1 = struct.unpack('<H',data1)[0]
                values2 = struct.unpack('<H',data2)[0]-50
                uwb1 = values1
                uwb2 = values2
                uwblist.append([uwb1,uwb2])
                #return uwb1, uwb2

U = Thread(target=UWB)
U.start()

sensors = [
    Sensor(board.D8,board.D7),#24/26
    Sensor(board.D25,board.D26), #22/37
    Sensor(board.D13,board.D19), #33/35
    Sensor(board.D23,board.D22), #16/15
    #Sensor(board.D6,board.D12)    

try:
    while True:
            distances = [sensor.getDistance() for sensor in sensors]
            print(" | ".join("{:.0f}cm".format(distance) for distance in distances))
            print(uwb1, uwb2)
            #time.sleep(0.1)
            forward(12)
            time.sleep(0.2)
            backward(12)
            time.sleep(0.2)

finally:
    stop()
    pca.deinit()   
    for sensor in sensors:
        del sensor