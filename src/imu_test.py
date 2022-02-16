#!/usr/bin/env python

import time
import serial

print("Starting TEST")

s = serial.Serial("/dev/idmind-artemis", baudrate=115200, timeout=1)

while True:
    aux = s.readline()
    if len(aux.split(",")) == 16:
        break
    else:
        # print("Initiating - {}".format(len(aux.split(","))))
        continue

print("Entering menu")
print("Written: {}".format(s.write("a\r\n")))
time.sleep(0.5)
#while True:
#    aux = s.readline()
#    if len(aux) == 0:
#        break
#    else:
#        print(aux),
#print("")

# print("Entered Menu")
print("Written: {}".format(s.write("1\r\n")))
time.sleep(0.5)
#while True:
#    aux = s.readline()
#    if len(aux) == 0:
#        break
#    else:
#        print(aux),
#print("")

print("Written: {}".format(s.write("4\r\n")))
time.sleep(0.5)
#while True:
#    aux = s.readline()
#    if len(aux) == 0:
#        break
#    else:
#        print(aux),
#print("")

s.write('10\r\n')
time.sleep(0.5)
#while True:
#    aux = s.readline()
#    if len(aux) == 0:
#        break
#    else:
#        print(aux),
#print("")

s.write("x\r\n")
time.sleep(0.5)
s.write("x\r\n")
time.sleep(0.5)
print("Resuming...")
while True:
    aux = s.readline()
    if len(aux.split(",")) == 16:
        print("Done")
        break
    else:
        # print("Initiating - {}".format(len(aux.split(","))))
        continue
