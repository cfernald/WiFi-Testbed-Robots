#!/usr/bin/env python
import rospy
import time
import argparse
from std_msgs.msg import String
from traxxas_node.msg import AckermannDriveMsg
import sys, select, termios, tty

speed = .5
turn = 70

def moveForward(seconds):
	start = time.time()
	print start
	time.clock()
	elapsed = 0
	while elapsed < seconds and not rospy.is_shutdown():
		print"Moving Forward"
		elapsed = time.time() - start
		ackermannDriveMsg = AckermannDriveMsg()
		ackermannDriveMsg.speed = 1*speed
		ackermannDriveMsg.steering_angle = 0*turn
		rospy.loginfo("Publishing speed = %f, steering = %f", ackermannDriveMsg.speed, ackermannDriveMsg.steering_angle)
		pub.publish(ackermannDriveMsg)

def moveBackward(seconds):
	start = time.time()
	print start
	time.clock()
	elapsed = 0
	while elapsed < seconds and not rospy.is_shutdown():
		print"Moving Backward"
		elapsed = time.time() - start
		ackermannDriveMsg = AckermannDriveMsg()
		ackermannDriveMsg.speed = -1*speed
		ackermannDriveMsg.steering_angle = 0*turn
		rospy.loginfo("Publishing speed = %f, steering = %f", ackermannDriveMsg.speed, ackermannDriveMsg.steering_angle)
		pub.publish(ackermannDriveMsg)



def stop(seconds):
	start = time.time()
	time.clock()
	elapsed = 0
	while elapsed < seconds and not rospy.is_shutdown():
		print"please stoppp"
		elapsed = time.time()-start
		ackermannDriveMsg = AckermannDriveMsg()
		ackermannDriveMsg.speed = 0
		ackermannDriveMsg.steering_angle = 0*turn
		rospy.loginfo("Publishing speed = %f, steering = %f", ackermannDriveMsg.speed, ackermannDriveMsg.steering_angle)
		pub.publish(ackermannDriveMsg)

def turnRight(seconds):
	start = time.time()
	time.clock()
	elapsed = 0
	while elapsed < seconds and not rospy.is_shutdown():
		print"Turning Right"
		elapsed = time.time() - start
		ackermannDriveMsg = AckermannDriveMsg()
		ackermannDriveMsg.speed = 1*speed
		ackermannDriveMsg.steering_angle = -1*turn
		rospy.loginfo("Publishing speed = %f, steering = %f", ackermannDriveMsg.speed, ackermannDriveMsg.steering_angle)
		pub.publish(ackermannDriveMsg)

def turnLeft(seconds):
	start = time.time()
	time.clock()
	elapsed = 0
	while elapsed < seconds and not rospy.is_shutdown():
		print"Turning Left"
		elapsed = time.time() - start
		ackermannDriveMsg = AckermannDriveMsg()
		ackermannDriveMsg.speed = 1*speed
		ackermannDriveMsg.steering_angle = 1*turn
		rospy.loginfo("Publishing speed = %f, steering = %f", ackermannDriveMsg.speed, ackermannDriveMsg.steering_angle)
		pub.publish(ackermannDriveMsg)



if __name__=="__main__":

   	settings = termios.tcgetattr(sys.stdin)

	seconds = 2
	speed = .5
	inc = 2
	wait = 5
	print 'Time:', seconds
	print 'Speed:', speed
	print 'Increment:', inc
	print 'Wait:', wait

	pub = rospy.Publisher('/traxxas_node/ackermann_drive', AckermannDriveMsg)
	rospy.init_node('star_route')


	status = 0

	try:
		while not rospy.is_shutdown():
#			print"Begin Moving"
#			time.sleep(wait)
#			print"BBBBBBBBBBBBBBB"
#			moveBackward(seconds)
#			time.sleep(wait)
#			print"FFFFFFFFFFFFFF"
#			moveForward(seconds)
#			print"please stopppppppppppppppp"
#			stop(seconds)
#			time.sleep(wait)
			turnLeft(999)
			moveForward(seconds)
#			time.sleep(wait)
#			turnRight(seconds)
			seconds += inc
	except:
		print e

	finally:
		ackermannDriveMsg = AckermannDriveMsg()
		#twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		#twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(AckermannDriveMsg)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
