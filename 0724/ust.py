import busio
import digitalio
import board
import keyboard

from board import SCL_1, SDA_1
from adafruit_pca9685 import PCA9685

from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2 # ver 3.4.18.65
import sys
import json
import numpy as np # ver 1.19.4

import serial
import struct
from threading import Thread
from threading import Lock

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

sensor1_distance = 0
sensor2_distance = 0
sensor3_distance = 0
sensor4_distance = 0

distance_lock = Lock()

def read_us_sensor(sensor, sensor_num):
    global sensor1_distance, sensor2_distance, sensor3_distance, sensor4_distance
    while True:
        distance = sensor.getDistance()
        with distance_lock:
            if sensor_num == 1:
                sensor1_distance = distance
            elif sensor_num == 2:
                sensor2_distance = distance
            elif sensor_num == 3:
                sensor3_distance = distance
            elif sensor_num == 4:
                sensor4_distance = distance

sensor1 = Sensor(board.D8, board.D7)
sensor2 = Sensor(board.D25, board.D26)
sensor3 = Sensor(board.D13, board.D19)
sensor4 = Sensor(board.D23, board.D22)

us_thread1 = Thread(target=read_us_sensor, args=(sensor1,1))
us_thread2 = Thread(target=read_us_sensor, args=(sensor2,2))
us_thread3 = Thread(target=read_us_sensor, args=(sensor3,3))
us_thread4 = Thread(target=read_us_sensor, args=(sensor4,4))

us_thread1.start()
us_thread2.start()
us_thread3.start()
us_thread4.start()

while True:
    print("Sensor 1 distance:", sensor1_distance)
    print("Sensor 2 distance:", sensor2_distance)
    print("Sensor 3 distance:", sensor3_distance)
    print("Sensor 4 distance:", sensor4_distance)
    time.sleep(0.1)