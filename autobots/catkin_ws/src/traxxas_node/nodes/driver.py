#!/usr/bin/env python

'''
Implements a ROS node that interfaces with the Traxxas
Stampede mobility plane used by the Pharos Lab at UT
Austin.

Author: Chien-Liang Fok
Date: 02/08/2012
'''

import roslib; roslib.load_manifest('traxxas_node')
import rospy, serial, sys, binascii, time, struct
from datetime import datetime
from std_msgs.msg import String
from threading import Thread
from traxxas_node.msg import AckermannDriveMsg
from traxxas_node.msg import AckermannMonitorMsg

''' Constant Defintions '''
PROTEUS_START = 0x24

'''
Computes the checksum of a tuple or list of data.

Input:
 - data: a list or tuple of data

Output: 
  The checksum (integer) which is the XOR of each byte in data.
'''
def computeChecksum(data):
	#print "Computing checksum of " + bytesToString(data)
	checksum = 0
	for b in data:
		#result = " Xoring 0x" + binascii.b2a_hex(chr(b)) + ", checksum = 0x" + binascii.b2a_hex(chr(checksum))
		checksum ^= b
		#print result + ", result = 0x" + binascii.b2a_hex(chr(checksum))
	return checksum

'''
Converts an array of bytes into a hexidecimal string.

Input:
 - bytes: an array of bytes (each value in it must be between 0 and 255).

Output: 
  The string representation of the array in hexidecimal format
'''
def bytesToString(bytes):
	result = "["
	for i in bytes:
		c = binascii.b2a_hex(chr(i)) 
		result += "0x" + c + ", "
	return result[:-2] + "]"

'''
SerialMonitor operates as a separate thread that 
receives incoming data from the Traxxas.  Since
status messages are transmitted at 10Hz, this
thread loops at 20Hz.
'''
class SerialMonitor(Thread): # SerialMonitor extends Thread
	''' 
	The constructor of SerialMonitor.

	Parameters:
	 - ser: The serial port object
         - watchdog: A list object that when non-empty allows the
           SerialMonitor's loop to exit.
	'''
	def __init__(self, ser):
		Thread.__init__(self, name="SerialMonitor") # Call superclass constructor
		self.ser = ser;
			
	def run(self):
                pub = rospy.Publisher('traxxas_node/ackermann_monitor', AckermannMonitorMsg)
		rospy.loginfo(rospy.get_name() + " SerialMonitor: Thread starting.")
		while not rospy.is_shutdown():
			while (ser.inWaiting() >= 18):
				startByte = ord(ser.read(1))  # need to use ord(...) to convert '$' to 0x24
				if (startByte == PROTEUS_START):
					# Read (size of status message) - 1 since start byte already received.
					rxdata = ser.read(17)
					rxdata = struct.unpack("B"*17, rxdata) # convert from string to tuple (an immutable sequence type)
					rxdata = [i for i in rxdata] # convert from tuple to list (a mutable sequence type)
					rxdata.insert(0, PROTEUS_START)
					
					#print "Received " + bytesToString(rxdata)
					
					checksum = computeChecksum(rxdata[:-1])
					
					#print "checksum = 0x" + binascii.b2a_hex(chr(checksum))
					
					# Format chars: http://docs.python.org/library/struct.html#format-characters
					resp = struct.unpack("<BhhHhhhhHB", struct.pack("B"*18, *rxdata))
					if (checksum == resp[len(resp)-1]):  # The last element in the tuple is the checksum
						#c = 1
						rospy.logdebug(rospy.get_name() + " SerialMonitor: Status: target speed = %i, current speed = %i, motor cmd = %i, prev err = %i, total err = %i, target steering angle = %i, current steering angle = %i, steering angle cmd = %i", resp[1], resp[2], resp[3], resp[4], resp[5], resp[6], resp[7], resp[8])

						#sys.stdout.write("target speed = " + str(resp[1]) \
						#	+ ", current speed = " + str(resp[2]) \
						#	+ ", motor cmd = " + str(resp[3]) \
						#	+ ", prev err = " + str(resp[4]) \
						#	+ ", total err = " + str(resp[5]) \
						#	+ ", target steering angle= " + str(resp[6]) \
						#	+ ", current steering angle = " + str(resp[7]) \
						#	+ ", streering angle cmd = " + str(resp[8]) \
						#	+ "\n")
						#sys.stdout.write("Received " + str(struct.unpack("<hhHhh", rxdata)) + "\n\n")
						#sys.stdout.write("Received " + str(struct.unpack("b"*10, rxdata)) + "\n\n")
						#sys.stdout.flush()
						ackermannMonitorMsg = AckermannMonitorMsg()
                        			ackermannMonitorMsg.speed = resp[2]
                        			ackermannMonitorMsg.angle = resp[7]
                        			pub.publish(ackermannMonitorMsg)

					else:
						rospy.logerr(rospy.get_name() + " SerialMonitor: Checksum mismatch 0x%s != 0x%s", \
							binascii.b2a_hex(chr(checksum)), binascii.b2a_hex(chr(resp[5])))
				else:
					rospy.logwarn(rospy.get_name() + " SerialMonitor: Invalid start byte 0x%s != 0x%s", \
						hex(startByte), binascii.b2a_hex(chr(PROTEUS_START)))

			time.sleep(0.1)  # cycle at 10Hz

'''
CommandSender periodically sends a move command to the Traxxas.
This is to prevent its safety stop from being triggered.
'''
class CommandSender(Thread): # CommandSender extends Thread
	
	''' 
	The constructor of CommandSender.

	Parameters:
	 - ser: The serial port object
	 - watchdog: A list object that when non-empty allows the
       SerialMonitor's loop to exit.
	'''
	def __init__(self, ser):
		Thread.__init__(self, name="CommandSender") # Call superclass constructor
		self.ser = ser;
		self.steering = 0
		self.speed = 0
		self.lastCmd = datetime.now()
	
	def driveCmdHandler(self, driveMsg):
		self.steering = int(driveMsg.steering_angle * 10)  # Convert to 1/10 degrees
		self.speed = int(driveMsg.speed * 100) # Convert to cm/s
		self.lastCmd = datetime.now()
		#rospy.loginfo(rospy.get_name() + " New command: Steering=%i 1/10 deg, speed=%i cm/s", \
		#	self.steering, self.speed)
	
	#def stop():
		#running = False

	def run(self):
		rospy.loginfo(rospy.get_name() + " CommandSender: Thread starting.")
		while not rospy.is_shutdown():
			# Check for stop condition (no cmd received)
			now = datetime.now()
			elapsed = now - self.lastCmd
			elapsed = float(elapsed.seconds) + elapsed.microseconds/1000000  # convert to seconds
			if elapsed > 0.4:
				#rospy.logwarn(rospy.get_name() + " CommandSender: Command absence threshold exceeded, stopping robot.")
				self.steering = 0
				self.speed = 0
			
			data = [PROTEUS_START, self.steering, self.speed]
			check = struct.unpack("BBBBB", struct.pack("<Bhh", *data))  # convert to byte tuple
			data.append(computeChecksum(check))
			bytes = struct.pack("<BhhB", *data)
			
			# Print what's being transmitted to the robot
			#print "CommandSender: sending: " + str(data) + " = " + bytesToString(bytes)

			numTX = ser.write(bytes)
			#rospy.loginfo(rospy.get_name() + " CommandSender: Sent %i bytes: %s, Steering=%i 1/10 deg, speed=%i cm/s", numTX, str(data), self.steering, self.speed)
			time.sleep(0.2)  # sleep for 0.2 seconds to cycle at 5Hz



def getSteering(self):
	rospy.loginfo(rospy.get_name() + " getSteering(): %i", __steering)
	return self.__steering

def getSpeed(self):
	rospy.loginfo(rospy.get_name() + " getSpeed(): %i", __speed)
	return self.__speed

if __name__ == '__main__':
	rospy.init_node('traxxas_driver')

	port = rospy.get_param('/traxxas_node/port', "/dev/ttyUSB0")
	baud = rospy.get_param('/traxxas_node/baud', 9600)
	ser = serial.Serial(port, baud)

	if (ser):
        	rospy.loginfo("Serial port " + ser.portstr + " opened.")
	else:
		rospy.logerr("Unable to open serial port")
		sys.exit()

	rospy.loginfo("Waiting 2s for Arduino to initialize...")
	time.sleep(2)

	ser.flushInput()

	# Create and start a SerialMonitor thread
	smThread = SerialMonitor(ser)
	smThread.start()

	# Create and start the CommandSender thread
	cmdThread = CommandSender(ser)
	cmdThread.start()

	
	rospy.Subscriber("traxxas_node/ackermann_drive", AckermannDriveMsg, cmdThread.driveCmdHandler)

	rospy.spin()
