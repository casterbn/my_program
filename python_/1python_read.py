
# -*- coding: utf-8 -*

import serial
import time
import thread
import Queue
import ring_buffer
import ctypes  
ll = ctypes.cdll.LoadLibrary   
lib = ll("./dch_test.so")    
var = 'dch'
var1 = 'csv'
lib.getTimeString(var,32,var1)

from ring_buffer import handle_buff_data
uart_ring_buff = Queue.Queue()
uart_buff = 0
	
ser = serial.Serial(

    port = '/dev/ttyAMA0',

    baudrate = 115200,

    parity = serial.PARITY_NONE,

    stopbits = serial.STOPBITS_ONE,

    bytesize = serial.EIGHTBITS,

    timeout = 1

    )


longBuff = 0
longIndex = 0
framIndexBuff = 0
framNum = 0;
TotalRxCnt = 0;
uart_buff = 0
def handle_uart_data( threadName, delay):
   count = 0
   while 1:
		time.sleep(delay)
		count += 1
		#print "%s: %s" % ( threadName, time.ctime(time.time()) )
		#findFrame(longBuff,longIndex,framIndexBuff,&framNum);
		read_uart_buff_data(uart_ring_buff,uart_buff)
		print(uart_buff)
# 创建数据处理线程
try:
   thread.start_new_thread( handle_uart_data, ("handle_uart_data_thread", 2, ) )
except:
   print "Error: unable to start thread"


def read_uart_buff_data(src, dst):
	dst = src



def main():

    while True:

        count = ser.inWaiting()

        if count != 0:

            recv = ser.read(count)
            uart_ring_buff.put(recv)
            #ring_buffer.handle_buff_data(recv,'1')
            #ser.write(uart_ring_buff.queue)
            #print(uart_ring_buff.queue)

        ser.flushInput()

        time.sleep(0.1)

 

if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:

        if ser != None:

            ser.close()
