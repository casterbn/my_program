# -*- coding:utf-8 -*-
#将所有TXT连接成一个，并删除掉重复记录
import os
import time

def replace(rootDir):
    print "start..."
    fh = open("array_save.txt", "w")
    fh.write("{ ")
    file = open("d3.txt", 'r')
    str = file.read()
    str_d3 = str.replace("D3", "0xD3");
    str_to_write = str_d3.replace(" ", " ,0x");
    fh.write(str_to_write)
    fh.write("};")		
    fh.close()
    print "end..."
replace('./')