#!/usr/bin/env python

'''
Sends a test movement command to the Traxxas.
Makes the robot move forward at 1m/s with a 
steering angle of 0 degrees.

Author: Chien-Liang Fok
Date: 02/08/2012
'''

import serial, sys, binascii, time, struct

PROTEUS_START = 0x24

ser = serial.Serial("/dev/ttyUSB0", 115200)

if (ser):
        print("Serial port " + ser.portstr + " opened.")

ser.flushInput()

print("Waiting two seconds for Arduino to start.")
time.sleep(2)

data = (PROTEUS_START, 0, 100, 0x40) # Move forward at 100cm/s with 0 degree steering angle
bytes = struct.pack("<BhhB", *data)

#data = (0x24, 0, 0, 0x64, 0, 0x40) # Move forward at 100cm/s
#bytes = struct.pack("BBBBBB", *data)


# Transmit the message at 10Hz to prevent the safety
# stop from being triggered.
while True:
	numTX = ser.write(bytes)
	sys.stdout.write("Wrote " + str(data) + ", num bytes: " + str(numTX) + "\n")
	sys.stdout.flush()

	while (ser.inWaiting() >= 12):
		startByte = ser.read(1)
		if (startByte == chr(PROTEUS_START)):
			#sys.stdout.write("Receiving 11 bytes...\n")
			rxdata = ser.read(11)
					
			# Format chars: http://docs.python.org/library/struct.html#format-characters
			respBytes = struct.unpack("<" + "B"*11, rxdata)
			#print "Received " + str(respBytes)
			checksum = PROTEUS_START
			for b in respBytes[:-1]:
				checksum ^= b
				#print "  after XORing " + str(b) + " checksum = " + str(checksum) + \	
				#	" (0x" + binascii.b2a_hex(chr(checksum)) + ")"

			resp = struct.unpack("<hhHhhB", rxdata)
			if (chr(checksum) == chr(resp[5])):
				sys.stdout.write("target speed = " + str(resp[0]) \
					+ ", current speed = " + str(resp[1]) \
					+ ", motor cmd = " + str(resp[2]) \
					+ ", prev err = " + str(resp[3]) \
					+ ", total err = " + str(resp[4]) \
					+ "\n")
				#sys.stdout.write("Received " + str(struct.unpack("<hhHhh", rxdata)) + "\n\n")
				#sys.stdout.write("Received " + str(struct.unpack("b"*10, rxdata)) + "\n\n")
				sys.stdout.flush()
			else:
				print "Checksum mismatch 0x" + binascii.b2a_hex(chr(checksum)) + " != 0x" \
					+ binascii.b2a_hex(chr(resp[5]))
		else:
			print "Invalid start byte 0x" + binascii.b2a_hex(startByte) + " != 0x" + \
				binascii.b2a_hex(chr(PROTEUS_START)) + ", discarding"

	time.sleep(0.1)
