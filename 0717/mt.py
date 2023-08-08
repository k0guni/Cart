import time
import busio
import cv2 # ver 3.4.18.65
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

while 1 :
    stop()
    print("act:")
    act = input()
#	print("spd:")
#	spd = input()

    if act == 'f':
        #start_time = time.time
        forward(30)
        # while True:
        #     time.sleep(0.01)
        #     key = cv2.waitKey(1) & 0xFF
        #     if key == ord("q"):
        #         break
        # end_time = time.time
        
        time.sleep(2)
        stop()
        #recordtime = end_time - start_time
        #print(recoretime)
    elif act == 'w':
        #start_time = time.time
        forward(10)
        # while True:
        #     time.sleep(0.01)
        #     key = cv2.waitKey(1) & 0xFF
        #     if key == ord("q"):
        #         break
        # end_time = time.time
        
        time.sleep(2)
        stop()
        #recordtime = end_time - start_time
        #print(recoretime)
    elif act == 'b':
        # for i in range(1,13,3):
        #     backward(i)
        backward(10)
        time.sleep(2)
        stop()
    elif act == 'r':
        for i in range(1,13,3):
            right(i)
        right(6)
        time.sleep(1)
        stop()
    elif act == 'l':
        for i in range(1,13,3):
            left(i)
        left(6)
        time.sleep(1)
        stop()
    elif act == 'c':
        for i in range(1,13,3):
            cw(i)
        cw(12)
        time.sleep(2)
        stop()
    elif act == 'w':
        for i in range(1,13,3):
            ccw(i)
        ccw(12)
        time.sleep(2)
        stop()
    elif act == 's':
        stop()
        time.sleep(1)
    elif act == 'z':
        z(24)
        time.sleep(2)
        stop()
    elif act == 'x':
        x(24)
        time.sleep(2)
        stop()
    elif act == 'n':
        n(24)
        time.sleep(2)
        stop()
    elif act == 'm':
        m(24)
        time.sleep(2)
        stop()

pca.deinit()
