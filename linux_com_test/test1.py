#!/usr/bin/python
import serial,time,thread

ser=serial.Serial('/dev/ttyS1', timeout=1)
print ser.portstr

def recv_func(sec):
    global ser
    print 'recv'
    while True:
        readbuff=ser.read(10)
        print ('recv ',readbuff,'\n')
        time.sleep(sec)

if __name__ == '__main__':
    thread.start_new_thread(recv_func,(2,))
#    thread.start_new(recv_func())
    print 'main'
    i = 0
    try:
        while True:
            ser.write(b'hello')
            time.sleep(2)
            i += 1
            print i
    finally:
        ser.close()
