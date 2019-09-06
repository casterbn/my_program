from bluetooth import *
import time
import bluetooth
import binascii

print "performing inquiry..."




device_num = 0
nearby_devices = discover_devices(lookup_names = True)

print "found %d devices" % len(nearby_devices)
print nearby_devices[0][0]
print "next step"
for name, addr in nearby_devices:
	
	print " %d %s - %s" % (device_num, addr, name)
	device_num+= 1

print nearby_devices

print "select bt device"
set_device_num = raw_input()

print "the set device %s" % nearby_devices[int(set_device_num)][0]

"""
nearby_devices = bluetooth.discover_devices(lookup_names=True)

for addr, name in nearby_devices:

    print("  %s - %s" % (addr, name))

 

    services = bluetooth.find_service(address=addr)

    for svc in services:

        print("Service Name: %s"    % svc["name"])

        print("    Host:        %s" % svc["host"])

        print("    Description: %s" % svc["description"])

        print("    Provided By: %s" % svc["provider"])

        print("    Protocol:    %s" % svc["protocol"])

        print("    channel/PSM: %s" % svc["port"])

        print("    svc classes: %s "% svc["service-classes"])

        print("    profiles:    %s "% svc["profiles"])

        print("    service id:  %s "% svc["service-id"])

        print("")
"""
	 
port = 31
#bd_addr = "20:54:FA:B4:8A:D0"
bd_addr = "80:7D:3A:B5:89:1E"
# Create the client socket
client_socket = bluetooth.BluetoothSocket( RFCOMM )

client_socket.connect((nearby_devices[int(set_device_num)][0], 1))

#client_socket.send("Hello World")

print "Finished"

print "file name:"
file_name = raw_input()
file = open(file_name,"wb")
while True:
#	time.sleep(1)
#	client_socket.send("Hello World!")
#	data = client_socket.recv(1024)
#	print data
	data = client_socket.recv(1024)
	file.write(data)
#	print data_bin
#	for element in data:
#           print('%#03x'%ord(element)),

client_socket.close()





#service

 
"""
#server_sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )

server_sock = BluetoothSocket( RFCOMM )


port = 1

server_sock.bind(("",port))

server_sock.listen(1)

 

client_sock,address = server_sock.accept()

print "Accepted connection from ",address

 

data = client_sock.recv(1024)

print "received [%s]" % data

 

client_sock.close()

server_sock.close()
"""
