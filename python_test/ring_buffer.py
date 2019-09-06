import serial
import time
import thread
import Queue
import string











def handle_buff_data(src, dst):
    #s='nihao,nihao,hello'
    #t='n'
    #result = s.find(t)
    result = src.count(dst) 
    print ('result = %d',result)

def main():
    handle_uart_data('123','456')



if __name__ == '__main__':
    main()
