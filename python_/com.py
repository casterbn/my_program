#serial0 ->ttyAMA0

import serial

import time

ser = serial.Serial(

    port = '/dev/ttyAMA0',

    baudrate = 115200,

    parity = serial.PARITY_NONE,

    stopbits = serial.STOPBITS_ONE,

    bytesize = serial.EIGHTBITS,

    timeout = 1

    )

#counter = "asdfasdf"

counter = 0
count = 0

# -*- coding:utf-8 -*-







for i in range(1,11):

    print counter

    ser.write("Write : %d " %(counter))

    #ser.write(counter)

    time.sleep(1)

    counter +=1

ser.close()



def main():

    while True:

        count = ser.inWaiting()  

        if count != 0:

            recv = ser.read(count)  

            ser.write("Recv some data is : ".encode("utf-8"))  

            ser.write(recv)  

            ser.flushInput()

        #time.sleep(0.1)  

 

if __name__ == '__main__':

    main()

    
