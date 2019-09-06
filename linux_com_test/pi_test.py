
import serial

 

t = serial.Serial('com12',9600)

print t.portstr

strInput = raw_input('enter some words:')

n = t.write(strInput)

print n

str = t.read(n)

print str
