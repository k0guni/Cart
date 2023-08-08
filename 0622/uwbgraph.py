#uwb
import serial
import struct
import threading
import matplotlib.pyplot as plt
import time

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

        # UWBThread 초기화
port = '/dev/ttyTHS1'
baudrate = 115200
uwb_thread = UWBThread(port, baudrate)
uwb_thread.start()

# 데이터 수집 시간 설정
data_collection_time = 10  # 10초 동안 데이터 수집

# 데이터 수집 시작 시간 저장
start_time = time.time()

# 데이터 수집
while time.time() - start_time < data_collection_time:
    pass

# UWBThread 정지
uwb_thread.stop()

# 그래프 그리기
time_values = range(len(uwb_thread.values1))  # 시간축
plt.plot(time_values, uwb_thread.values1, label='Value 1')
plt.plot(time_values, uwb_thread.values2, label='Value 2')
plt.xlabel('Time')
plt.ylabel('Values')
plt.legend()
plt.show()
