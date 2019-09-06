#!/usr/bin/python

def change_list(list):
    list.append([1,2])
    list.append(1)
    print "new",list
    return
my_list = [10,20]
change_list(my_list)
print "now",my_list

