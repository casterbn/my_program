#!/usr/bin/python
import serial
portx="COM77"
bps=115200
timex=5
ser=serial.Serial(portx,bps,timeout=timex)
while 1:
	print ser.readline()
	#print ser.read_all()
ser.close()