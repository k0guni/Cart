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
        timeout = 1
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


def motor(speed, i, c):

    if speed > 100:
        speed = 100
    elif speed < -100:
        speed = -100
    duty_cycle_value = abs(speed*(0xffff//100))
    if speed == 0:
        pca.channels[dir_channel+4*i].duty_cycle = int(0xffff)
        pca.channels[brk_channel+4*i].duty_cycle = int(0xffff)
    elif speed > 0:
        pca.channels[dir_channel+4*i].duty_cycle = int(0xffff)
        pca.channels[brk_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[pwm_channel+4*i].duty_cycle = int(duty_cycle_value)
        # if c == 0:
        #     pca.channels[pwm_channel+4*i].duty_cycle = 0x1fff # 8191
        # elif c == 1:
        #     pca.channels[pwm_channel+4*i].duty_cycle = 0x17ff # 6143 
        # elif c == 2:
        #     pca.channels[pwm_channel+4*i].duty_cycle = 0x07ff # 2047
    else:
        pca.channels[dir_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[brk_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[pwm_channel+4*i].duty_cycle = int(duty_cycle_value)
        # if c == 0:
        #     pca.channels[pwm_channel+4*i].duty_cycle = 0x1fff
        # elif c == 1:
        #     pca.channels[pwm_channel+4*i].duty_cycle = 0x17ff
        # elif c == 2:
        #     pca.channels[pwm_channel+4*i].duty_cycle = 0x07ff

def forward(speed):
    motor(-speed,0,0)
    motor(-speed,1,0)
    motor(speed,2,0)
    motor(speed,3,0)
def backward(speed):
    motor(speed,0,0)
    motor(speed,1,0)
    motor(-speed,2,0)
    motor(-speed,3,0)
def right(speed):
    motor(-speed,0,0)
    motor(speed,1,0)
    motor(-1*-speed,2,0)
    motor(-1*speed,3,0)
def left(speed):
    motor(speed,0,0)
    motor(-speed,1,0)
    motor(-1*speed,2,0)
    motor(-1*-speed,3,0)
def ccw(speed):
    motor(-speed,0,1)
    motor(-speed,1,1)
    motor(-1*speed,2,1) 
    motor(-1*speed,3,1)
def cw(speed):
    motor(speed,0,1)
    motor(speed,1,1)
    motor(speed,2,1)
    motor(speed,3,1)
def stop():
    motor(0,0,0)
    motor(0,1,0)
    motor(0,2,0)
    motor(0,3,0)

class UWBThread(threading.Thread):
    def __init__(self, port, baudrate):
        super(UWBThread, self).__init__()
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.running = False

    def run(self):
        self.running = True
        while self.running:
            if self.ser.in_waiting > 5:
                id = self.ser.read()
                if id == b'\x14':
                    data1 = self.ser.read(2)
                    data2 = self.ser.read(2) 
                    values1 = struct.unpack('<H', data1)[0]
                    values2 = struct.unpack('<H', data2)[0] - 50
                    if ((values1 < 3000) and (values1 != 2580) and (values1 != 1300) and (values1 != 2068) and (values2 < 3000) and (values2 != 2580) and (values2 != 1300) and (values2 != 2068) and (values2 != 1812)):
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