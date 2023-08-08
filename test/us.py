
#2023-05-26
#TUK Mechatronics 2018130002 KoGeonHui
#diffrent - test + try,finally
#use for - 4 motor with 4 us 


import time
import busio
import digitalio
import board
import keyboard

from board import SCL_1, SDA_1
from adafruit_pca9685 import PCA9685

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

def main():
    sensors = [
        Sensor(board.D9,board.D10),#19/21
        #Sensor(board.D25,board.D26), #22/37
        #Sensor(board.D13,board.D19), #33/35
        #Sensor(board.D23,board.D22), #16/15
        #Sensor(board.D6,board.D12)         
    ]
    try:
        while True:
            distances = [sensor.getDistance() for sensor in sensors]
            print(" | ".join("{:.0f}cm".format(distance) for distance in distances))
            #time.sleep(0.1)
    finally: 
        for sensor in sensors:
            del sensor
        
if __name__ == "__main__":
    main()
