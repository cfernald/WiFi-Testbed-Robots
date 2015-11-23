#!/usr/bin/env python

'''
Sends a test movement command to the Traxxas.
Makes the robot move forward at 1m/s with a 
steering angle of 0 degrees.  Uses a separate
thread to receive response messages.

Author: Chien-Liang Fok
Date: 02/08/2012
'''

import serial, sys, binascii, time, struct
from threading import Thread

PROTEUS_START = 0x24

'''
SerialMonitor operates as a separate thread that 
receives incoming data from the Traxxas.
'''
class SerialMonitor(Thread): # SerialMonitor extends Thread

	''' 
	The constructor of SerialMonitor.

	Parameters:
	 - ser: The serial port object
         - watchdog: A list object that when non-empty allows the
           SerialMonitor's loop to exit.
	'''
	def __init__(self, ser, watchdog):
		Thread.__init__(self) # Call superclass constructor
		self.ser = ser;
		self.watchdog = watchdog
		
	def run(self):
		print "Serial Monitor thread starting."
		while(len(self.watchdog) == 0):
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

ser = serial.Serial("/dev/ttyUSB0", 115200)

if (ser):
        print("Serial port " + ser.portstr + " opened.")

ser.flushInput()

print("Waiting two seconds for Arduino to start...")
time.sleep(2)

watchdog = []

# Create and start a SerialMonitor thread
m = SerialMonitor(ser, watchdog)
m.start()

#data = (PROTEUS_START, 0, 100, 0x40) # Move at 100cm/s with 0 degree steering angle
#data = (PROTEUS_START, 200, 100, 0x88) # Move at 100cm/s with 200 1/10 degree steering angle
#data = (PROTEUS_START, -200, 100, 0x87) # Move at 100cm/s with -200 1/10 degree steering angle
data = (PROTEUS_START, -200, -100, 0x80) # Move at -100cm/s with -200 1/10 degree steering angle

bytes = struct.pack("<BhhB", *data)

#data = (0x24, 0, 0, 0x64, 0, 0x40) # Move forward at 100cm/s
#bytes = struct.pack("BBBBBB", *data)

try:
	# Transmit the message at 10Hz to prevent the safety
	# stop from being triggered.
	while True:
		numTX = ser.write(bytes)
		sys.stdout.write("Sent " + str(data) + ", num bytes: " + str(numTX) + "\n")
		sys.stdout.flush()
		time.sleep(0.1)  # sleep for 0.1 seconds
except (KeyboardInterrupt, SystemExit):
	print "Control-C detected, halting program"
	watchdog.append("quit")  # allow Monitor thread to exit
	sys.exit()
