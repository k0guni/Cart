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

def forrward(speed):
    motor(-(1.3*speed),0,0)
    motor(-(speed),1,0)
    motor(speed,2,0)
    motor(0.7*speed,3,0)
def forward(speed):
    motor(-(speed),0,0)
    motor(-(speed),1,0)
    motor(speed,2,0)
    motor(speed,3,0)
def bacckward(speed):
    motor((1.3*speed),1,0)
    motor(-1.3*speed,2,0)
    motor(-0.7*speed,3,0)
    motor((speed),0,0)
def backward(speed):
    motor((speed),1,0)
    motor(-speed,2,0)
    motor(-speed,3,0)
    motor((speed),0,0)
def right(speed):
    motor(-(speed),0,0)
    motor(speed,1,0)
    motor(-1*-(speed),2,0)
    motor(-1*(speed),3,0)
def left(speed):
    motor(((speed)),0,0)
    motor(-speed,1,0)
    motor(-1*(1.3*speed),2,0)
    motor(-1*-(speed),3,0)
def ccw(speed):
    motor(-1.2*speed,0,1)
    motor(-speed,1,1)
    motor(-1*speed,2,1) 
    motor(-0.8*speed,3,1)
def cw(speed):
    motor(1.2*speed,0,1)
    motor(speed,1,1)
    motor(speed,2,1)
    motor(0.8*speed,3,1)
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

        # while True:
        #     time.sleep(0.01)
        #     key = cv2.waitKey(1) & 0xFF
        #     if key == ord("q"):
        #         break
        # end_time = time.time
        # for i in range(1,13,1):
        #     forward(i)
        #     time.sleep(0.001)
        forward(20)
        time.sleep(2)
        stop()
        time.sleep(0.001)
        #recordtime = end_time - start_time
        #print(recoretime)
    elif act == 'b':
        # for i in range(1,13,3):
        #     backward(i)
        backward(20)
        time.sleep(2)
        stop()
    elif act == 'r':
        for i in range(1,13,3):
            right(i)
        right(20)
        time.sleep(1)
        stop()
    elif act == 'l':
        for i in range(1,13,3):
            left(i)
        left(20)
        time.sleep(1)
        stop()
    elif act == 'c':
        cw(24)
        time.sleep(2.2)
        stop()
    elif act == 'w':
        ccw(24)
        time.sleep(2.2)
        stop()
    elif act == 's':
        stop()
        time.sleep(1)
    elif act == 't':
        forward(20)
        time.sleep(8)
        ccw(24)
        time.sleep(2.1)
        stop()
        

pca.deinit()
