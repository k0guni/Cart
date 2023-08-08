import serial
import struct
import time
values1_buffer = []
values2_buffer = []

ser = serial.Serial('/dev/ttyTHS1',115200,timeout=1)
print("code start")
while True:
	if ser.in_waiting >= 5:
		id = ser.read()
		if id == b'\x14':
			data1 = ser.read(2)
			data2 = ser.read(2)
			values1 = struct.unpack('<H',data1)[0]
			values2 = struct.unpack('<H',data2)[0]
			if (values1 < 3000)and(values1 != 2580)and(values1 != 1300)and(values1 != 2068)and(values2 < 3000)and(values2 != 2580)and(values2 != 1300)and(values2 != 2068)and(values2 != 1812):
				#print("dist1:{},dist2:{}".format(values1,values2))
				#print("dist:{}".format(values1-values2))
				#print(values1)
	#moving average filter
				values1_buffer.append(values1)
				values2_buffer.append(values2)
				if len(values1_buffer) > 3:
					values1_buffer.pop(0)
				if len(values2_buffer) > 3:
					values2_buffer.pop(0)
					values1 = sum(values1_buffer) / len(values1_buffer)
					values2 = sum(values2_buffer) / len(values2_buffer)
					print("values1: {:.2f}, values2: {:.2f}".format(values1,values2))




