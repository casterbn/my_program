# -*- coding:utf-8 -*-
#将所有TXT连接成一个，并删除掉重复记录
import os
import time
def BianLi(rootDir):
    print "start..."
    start_time = time.time()
    fh = open("all.txt", "w")
    list1=[]
    for root,dirs,files in os.walk(rootDir):
#	for files in os.walk(rootDir):
		for filename in files:
			print filename
			result = "test" in filename
			if result != True:
				continue
			filepath = rootDir + filename
			file = open(filepath, 'r')
			for i in file:
				line = str(i).strip()
				print str(line)
				if line in list1:
					continue
				else:
					list1.append(line)
					fh.write(line+'\n')
    fh.close()
    end_time = time.time()
    print "end,%.2fs" % (end_time - start_time)
"""	
    for root,dirs,files in os.walk(rootDir):
        for filename in files:　　
			print filename
            filepath = rootDir+filename　　
            file = open(filepath, 'r')　　
            for i in file:　　#i即为file中的一行，不用再readline()了
                line = str(i).strip()
                print str(line)
                if line in list1:　　#判断list里面是否有这个记录了，如果没有就加入list，如果有就跳过
                    continue
                else:
                    list1.append(line)　　#向list里添加记录
                    fh.write(line+'\n')
    fh.close()
    end_time = time.time()
    print "全部数据拼接完毕，用时%.2f秒" % (end_time - start_time)
"""	
	
rootDir = '../'
BianLi(rootDir)