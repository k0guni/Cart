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

import busio
import digitalio
import board
import keyboard

class Sensor():
    def __init__(self, trig_pin, echo_pin):
        self.trig_pin = digitalio.DigitalInOut(trig_pin)
        self.echo_pin = digitalio.DigitalInOut(echo_pin)
        self.trig_pin.direction = digitalio.Direction.OUTPUT
        self.echo_pin.direction = digitalio.Direction.INPUT
    def getDistance(self):
        self.trig_pin.value = False
        self.trig_pin.value = True
        time.sleep(0.00001)        
        self.trig_pin.value = False

        signaloff = 0
        signalon = 0
        timeout = 30
        start_time = time.time()
        while self.echo_pin.value == 0:
            signaloff = time.time()
            # if signaloff - start_time > timeout:
            #     raise RuntimeError("Timeout waiting for signal off")
        while self.echo_pin.value == 1:
            signalon = time.time()
            # if signalon - start_time > timeout:
            #     raise RuntimeError("Timeout waiting for signal on")

        return (signalon - signaloff) * 17000

    def __del__(self):
        self.trig_pin.deinit()
        self.echo_pin.deinit()

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

sensors = [
    Sensor(board.D8,board.D7),#24/26
    Sensor(board.D25,board.D26), #22/37
    Sensor(board.D13,board.D19), #33/35
    Sensor(board.D23,board.D22), #16/15
    #Sensor(board.D6,board.D12)         
]

#plt.figure()
U = Thread(target=UWB)
U.start()
while True:
    if(uwb1 == 0)and(uwb2 == 0):
        print("pass")
        continue
    u1 = uwblist[-1][0]
    u2 = uwblist[-1][1]
    dif = u1 - u2 
    print("dist1:{},dist2:{}".format(u1,u2))
    distances = [sensor.getDistance() for sensor in sensors]
    print(" | ".join("{:.0f}cm".format(distance) for distance in distances))
    #print(dif)
    time.sleep(0.1)
#    plt.plot(num,u1,'rs--',num,u2,'bs--')        
#    plt.pause(0.01)


