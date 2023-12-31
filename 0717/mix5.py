#2023-07-13
#TUK Mechatronics 2018130002 KoGeonHui
#refer to - python 3.6.9
#use for - aruco detect pose, distance
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
        timeout = 5
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


def motor(speed, i):

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
    else:
        pca.channels[brk_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[dir_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[pwm_channel+4*i].duty_cycle = int(motor_speed)

def forward(speed):
    motor(-speed,0)
    motor(-speed,1)
    motor(speed,2)
    motor(speed,3)
def backward(speed):
    motor(speed,0)
    motor(speed,1)
    motor(-speed,2)
    motor(-speed,3)
def right(speed):
    motor(-speed,0)
    motor(speed,1)
    motor(-1*-speed,2)
    motor(-1*speed,3)
def left(speed):
    motor(speed,0)
    motor(-speed,1)
    motor(-1*speed,2)
    motor(-1*-speed,3)
def ccw(speed):
    motor(-speed,0)
    motor(-speed,1)
    motor(-1*speed,2) 
    motor(-1*speed,3)
def cw(speed):
    motor(speed,0)
    motor(speed,1)
    motor(speed,2)
    motor(speed,3)
def stop():
    motor(0,0)
    motor(0,1)
    motor(0,2)
    motor(0,3)

def UWB():
    global uwb1,uwb2,uwblist

    while True:
        if ser.in_waiting >= 5:
            id = ser.read()
            if id == b'\x14':
                data1 = ser.read(2)
                data2 = ser.read(2)
                values1 = struct.unpack('<H',data1)[0]
                values2 = struct.unpack('<H',data2)[0]-50
                uwb1 = values1
                uwb2 = values2
                uwblist.append([uwb1,uwb2])
                #return uwb1, uwb2
i2c = busio.I2C(SCL_1, SDA_1)

pca = PCA9685(i2c, address=0x40)
#pca.frquency max 1526 min 24 hz
pca.frequency = 100

pwm_channel = 0
dir_channel = 1
brk_channel = 2

ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
	default="DICT_ARUCO_ORIGINAL",
	help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

detectgray = True
drawaxes = True

if ARUCO_DICT.get(args["type"], None) is None:
	print("[INFO] ArUCo tag of '{}' is not supported".format(
		args["type"]))
	sys.exit(0)

print("[INFO] detecting '{}' tags...".format(args["type"]))
arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT["DICT_5X5_50"])
arucoParams = cv2.aruco.DetectorParameters_create()


with open('/home/ubuntu/ex_code/aruco/camera.json', 'r') as json_file:
	camera_data = json.load(json_file)
dist = np.array(camera_data["dist"])
mtx = np.array(camera_data["mtx"])

vs = VideoStream(src=0).start()
time.sleep(1)

frame = vs.read()

h, w = frame.shape[:2]

newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (h, w), 0, (h, w))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), cv2.CV_32FC1)
x, y, w1, h1 = roi
yh1 = y + h1
xw1 = x + w1

sensors = [
    #Sensor(board.D10,board.D9),#19/21
    #Sensor(board.D25,board.D26), #22/37
    #Sensor(board.D13,board.D19), #33/35
    #Sensor(board.D23,board.D22), #16/15
    #Sensor(board.D6,board.D12)         
]
A = 50 #카메라 추종거리
B = 30
R = 160
L = -160
try:
    while True:
        frame = vs.read()
        dst1 = cv2.remap
        if detectgray:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
        else:
            (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
        if len(corners) > 0:
            if drawaxes:
                print("catch")
                for i in range(0, len(ids)):
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, mtx, dist)
                    cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, 0.02) 
                    i += 1
                    #x: red y:green z:blue
                    R, _ = cv2.Rodrigues(rvec)
                    z_axis = R[:, 2]
                    x_axis = R[:, 0]
                    y_axis = R[:, 1]
                    rev = 180*np.cross(R[:, 0], R[:, 2])[2]	
                    distcam = np.linalg.norm(tvec)*178
                    time.sleep(0.5)
                    print("distcam = {}cm".format(distcam))                   
                    if distcam >= 70:
                        forward(10)
                    else:
                        stop()
      
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        else:
            stop()                        
    
        # key = cv2.waitKey(1) & 0xFF
        # if key == ord("q"):
        #     break
finally:
#    for sensor in sensors:
#        del sensor
    vs.stop()
    pca.deinit()

