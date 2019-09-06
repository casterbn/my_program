#!/usr/bin/python

fs = open("sensor_test_100.txt","r")
lines = fs.readlines()
#print lines
print lines[1]
#for i in range(len(lines))
#    print i
for line in lines:
    raw_input("next line:")
    str = line[0:5]
    print "str = ",str
    if cmp(str,"0_0_0") == 0 :
        print "this is sensor data"
    print line
