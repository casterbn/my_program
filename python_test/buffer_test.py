import serial
import time
import thread
import Queue
import string

class RingClass:
    
    def __init__(self, size_max):
        self.size = size_max
        self.data = []
    
    class __Full:  
        def append(self, x):
            self.data[self.cur] = x
            self.cur = (self.cur+1) % self.size 
               
        def tolist(self):
            return self.data[self.cur:]+self.data[:self.cur]
    
    def append(self, x):
        self.data.append(x)
        if len(self.data) == self.size:
            self.cur = 0
            self.__class__ = self.__Full
    def tolist(self):
        return self.data
		
if __name__ == "__main__":
    x = RingClass(5)
    x.append(1);x.append(2);x.append(3);x.append(4)
    print x.__class__,x.tolist()
    x.append(5)
    print x.__class__,x.tolist()
    x.append(6)
    print x.__class__, x.data, x.tolist()
    x.append(7);x.append(8);x.append(9);x.append(10)
    print x.__class__, x.data,x.tolist()