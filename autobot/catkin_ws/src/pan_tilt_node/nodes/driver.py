#!/usr/bin/env python

'''
Implements a ROS node that interfaces with an Arduino to control the pan and
tilt angle of a device.  It is used by the Pharos Lab at UT Austin.

Author: Chien-Liang Fok
Date: 02/20/2012
'''

import roslib; roslib.load_manifest('pan_tilt_node')
import rospy, serial, sys, binascii, time, struct
from datetime import datetime
from std_msgs.msg import String
from pan_tilt_node.msg import PanTiltCmd

''' Constant Defintions '''
PROTEUS_START = 0x24
SERVO_MIN_DEG = -90
SERVO_MAX_DEG = 90

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
		#print "Converting bytes to string: " + str(i)
		c = binascii.b2a_hex(chr(i)) 
		result += "0x" + c + ", "
	return result[:-2] + "]"

def degToServo(deg):
	result = int(-100 / 9 * deg + 1500)
	rospy.loginfo("Converting %f degrees to %i servo units", deg, result)
	return result
	
def panTiltCallback(panTiltCmd):
	# Convert the angles (in degrees) into servo units
	if panTiltCmd.pan_angle > SERVO_MAX_DEG:
		panTiltCmd.pan_angle = SERVO_MAX_DEG
	if panTiltCmd.pan_angle < SERVO_MIN_DEG:
                panTiltCmd.pan_angle = SERVO_MIN_DEG
	if panTiltCmd.tilt_angle > SERVO_MAX_DEG:
                panTiltCmd.tilt_angle = SERVO_MAX_DEG
        if panTiltCmd.tilt_angle < SERVO_MIN_DEG:
                panTiltCmd.tilt_angle = SERVO_MIN_DEG

	pan = degToServo(panTiltCmd.pan_angle)
	tilt = degToServo(panTiltCmd.tilt_angle)
	data = [PROTEUS_START, pan, tilt]
	check = struct.unpack("BBBBB", struct.pack("<Bhh", *data))  # convert to byte tuple
	data.append(computeChecksum(check))
	bytes = struct.pack("<BhhB", *data)
			
	# Print what's being transmitted to the robot
	print "Sending: " + str(data) + " = " + bytesToString(struct.unpack("B"*6, bytes))

	numTX = ser.write(bytes)
	#rospy.loginfo(rospy.get_name() + " CommandSender: Sent %i bytes: %s, Steering=%i 1/10 deg, speed=%i cm/s", numTX, str(data), self.steering, self.speed)
			

if __name__ == '__main__':
	rospy.init_node('pan_tilt_node')

	port = rospy.get_param('/pan_tilt_node/port', "/dev/ttyUSB0")
	baud = rospy.get_param('/pan_tilt_node/baud', 115200)
	ser = serial.Serial(port, baud)

	if (ser):
        	rospy.loginfo("Serial port " + ser.portstr + " opened.")
	else:
		rospy.logerr("Unable to open serial port")
		sys.exit()

	rospy.loginfo("Waiting 2s for Arduino to initialize...")
	time.sleep(2)

	ser.flushInput()
	
	rospy.Subscriber("pan_tilt_node/pan_tilt_cmd", PanTiltCmd, panTiltCallback)
	rospy.loginfo("Subscribed to topic pan_tilt_node/pan_tilt_cmd...")
	rospy.spin()
