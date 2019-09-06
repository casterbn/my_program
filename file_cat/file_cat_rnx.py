# -*- coding:utf-8 -*-
#将所有TXT连接成一个，并删除掉重复记录
import os
import time

def foreach(rootDir,file):
    print "start..."
    start_time = time.time()
#    fh = open("file_cat_rnx.txt", "w")
    fh = open(file, "w+")
    list1=[]
    for root,dirs,files in os.walk(rootDir):

#		for filename in files:
#			apath = os.path.join(root, filename)
#			print apath
#			continue

		for filename in files:
			print "for each file!!!!",filename
			result = key_world in filename
#			result = "046.antPOD.rnx" in filename			
			if result != True:
				continue
#			filepath = rootDir + filename
			filepath = os.path.join(root, filename)
			print filepath
			file = open(filepath, 'r')
			str_ori = file.read()
			str = str_ori.replace("EOF", " ")
#			print str
			str_to_write = str.split("END OF HEADER",1);
#			print "split:",str_to_write[0]		#spilt 0
#			file_start = str_to_write[0]
#			list1.append(line)
			if file_start == 0:
				global file_start
				fh.write(str)
				file_start+= 1
			else:
				fh.write(str_to_write[1])
    fh.close()
    end_time = time.time()
    print "end,%.2fs" % (end_time - start_time)
rootDir = '../'
file_start = 0
key_world = raw_input("please enter:")
txt_file = key_world + ".txt"
foreach(rootDir,txt_file)