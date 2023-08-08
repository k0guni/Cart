import serial
import struct
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
import threading
#graph
#import matplotlib.pyplot as plt

#1
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
	default="DICT_ARUCO_ORIGINAL",
	help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

class UWBThread(threading.Thread):
    def __init__(self, port, baudrate):
        super(UWBThread, self).__init__()
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.running = False

    def run(self):
        self.running = True
        while self.running:
            if self.ser.in_waiting >= 5:
                id = self.ser.read()
                if id == b'\x14':
                    data1 = self.ser.read(2)
                    data2 = self.ser.read(2) 
                    values1 = struct.unpack('<H', data1)[0]
                    values2 = struct.unpack('<H', data2)[0] - 50
                    #if ((values1 < 3000) and (values1 != 2580) and (values1 != 1300) and (values1 != 2068) and (values2 < 3000) and (values2 != 2580) and (values2 != 1300) and (values2 != 2068) and (values2 != 1812)):
                    print("dist1: {}, dist2: {}".format(values1, values2))
                    #moving average filter
                    # values1_buffer.append(values1)
                    # values2_buffer.append(values2)
                    # if len(values1_buffer) > 3:
                    #     values1_buffer.pop(0)
                    # if len(values2_buffer) > 3:
                    #     values2_buffer.pop(0)
                    # values1 = sum(values1_buffer) / len(values1_buffer)
                    # values2 = sum(values2_buffer) / len(values2_buffer)
                    # print("values1: ", values1, "values2", values2)
            time.sleep(0.1)
    def stop(self):
        self.running = False
        self.join()


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
time.sleep(2)

frame = vs.read()

h, w = frame.shape[:2]

newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (h, w), 0, (h, w))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), cv2.CV_32FC1)
x, y, w1, h1 = roi
yh1 = y + h1
xw1 = x + w1
#1

port = '/dev/ttyTHS1'
baudrate = 115200
values1_buffer = []
values2_buffer = []
prev_values1 = 0
prev_values2 = 0
loop = 0
uwb_thread = UWBThread(port, baudrate)
uwb_thread.start()

time.sleep(2)
while True:
    # aruco detect
    frame = vs.read()
    dst1 = cv2.remap
    if detectgray:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
    else:
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
    if len(corners) > 0:
        if drawaxes:
            for i in range(0, len(ids)):
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, mtx, dist)
                cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, 0.02) 
                #x: red y:green z:blue
                R, _ = cv2.Rodrigues(rvec)
                z_axis = R[:, 2]
                x_axis = R[:, 0]
                y_axis = R[:, 1]
                # rev == rotate angle
                rev = np.cross(R[:, 1], R[:, 2])[2]	
                print("rev: ", rev)
                distcam = np.linalg.norm(tvec)*178
                time.sleep(0.5)
                #print("distcam = {}cm".format(distcam))
        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow("Frame", frame)
    end_time = time.time() 
    loop += 1
    #print(loop)
uwb_thread.stop()
