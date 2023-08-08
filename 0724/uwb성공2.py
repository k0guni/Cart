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
import keyboard
import board
from board import SCL_1, SDA_1
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL_1, SDA_1)

pca = PCA9685(i2c, address=0x40)
pca.frequency = 100

pwm_channel = 0
dir_channel = 1
brk_channel = 2


def motor(speed, i, c):

    if speed > 100:
        speed = 100
    elif speed < -100:
        sped = -100
    motor_speed = abs(speed*(0xffff//100))
    if speed == 0:
        pca.channels[brk_channel+4*i].duty_cycle = int(0xffff)
        pca.channels[dir_channel+4*i].duty_cycle = int(0xffff)
    elif speed > 0:
        pca.channels[brk_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[dir_channel+4*i].duty_cycle = int(0xffff)
        pca.channels[pwm_channel+4*i].duty_cycle = int(motor_speed)
        # if c == 0:
        #     pca.channels[pwm_channel+4*i].duty_cycle = 0x2fff
        # elif c == 1:
        #     pca.channels[pwm_channel+4*i].duty_cycle = 0x11ff
        # elif c == 2:
        #     pca.channels[pwm_channel+4*i].duty_cycle = 0x07ff
    else:
        pca.channels[brk_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[dir_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[pwm_channel+4*i].duty_cycle = int(motor_speed)
        # if c == 0:
        #     pca.channels[pwm_channel+4*i].duty_cycle = 0x2fff
        # elif c == 1:
        #     pca.channels[pwm_channel+4*i].duty_cycle = 0x11ff
        # elif c == 2:
        #     pca.channels[pwm_channel+4*i].duty_cycle = 0x07ff

def forward(speed):
    motor(-int(1.2*speed),0,0)
    motor(-int(speed),1,0)
    motor(int(0.9*speed),2,0)
    motor(int(0.9*speed),3,0)
def backward(speed):
    motor(int(1.2*speed),1,0)
    motor(-int(speed),2,0)
    motor(-int(0.9*speed),3,0)
    motor(int(0.9*speed),0,0)
def right(speed):
    motor(-(int(1.2*speed)),0,0)
    motor(int(speed),1,0)
    motor(int(0.9*speed),2,0)
    motor(-int(0.9*speed),3,0)
def left(speed):
    motor(int(1.2*speed),0,0)
    motor(-int(speed),1,0)
    motor(-int(0.9*speed),2,0)
    motor(int(0.9*speed),3,0)
def ccw(speed):
    motor(-int(1.2*speed),0,1)
    motor(-int(speed),1,1)
    motor(-int(speed),2,1) 
    motor(-int(0.9*speed),3,1)
def cw(speed):
    motor(int(1.2*speed),0,1)
    motor(int(speed),1,1)
    motor(int(0.9*speed),2,1)
    motor(int(0.9*speed),3,1)
def stop():
    motor(0,0,0)
    motor(0,1,0)
    motor(0,2,0)
    motor(0,3,0)

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
        timeout = 10
        start_time = time.time()
        while self.echo_pin.value == 0:
            signaloff = time.time()
            if signaloff - start_time > timeout:
                raise RuntimeError("Timeout waiting for signal off")
        while self.echo_pin.value == 1:
            signalon = time.time()
            if signalon - start_time > timeout:
                raise RuntimeError("Timeout waiting for signal on")

        return (signalon - signaloff) * 17000
    def __del__(self):
        self.trig_pin.deinit()
        self.echo_pin.deinit()

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
    forward(12)
    #print(dif)
    time.sleep(0.1)
#    plt.plot(num,u1,'rs--',num,u2,'bs--')        
#    plt.pause(0.01)


