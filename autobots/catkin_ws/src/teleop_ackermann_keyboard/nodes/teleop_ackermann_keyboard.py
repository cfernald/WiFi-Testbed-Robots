#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_ackermann_keyboard')
import rospy

from traxxas_node.msg import AckermannDriveMsg

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
"""

moveBindings = {
		'i':(1,0,0),
		'o':(1,-1,0),
		'j':(0,1,0),
		'l':(0,-1,0),
		'u':(1,1,0),
		',':(-1,0,0),
		'.':(-1,-1,0),
		'm':(-1,1,0),
		'f':(0,0,1),
		'v':(0,0,-1)
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .5
turn = 20

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('/traxxas_node/ackermann_drive', AckermannDriveMsg)
	rospy.init_node('teleop_ackermann_keyboard')

	x = 0
	z = 0
	th = 0
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				th = moveBindings[key][1]
				z = moveBindings[key][2]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:
				x = 0
				z = 0
				th = 0
				if (key == '\x03'):  # end of text char
					break

			ackermannDriveMsg = AckermannDriveMsg()
			ackermannDriveMsg.speed = x*speed;
			ackermannDriveMsg.steering_angle = th*turn 
			#twist.linear.y = 0; twist.linear.z = z*speed
			#twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			rospy.loginfo("Publishing speed = %f, steering = %f", ackermannDriveMsg.speed, ackermannDriveMsg.steering_angle)		
			pub.publish(ackermannDriveMsg)

	except:
		print e

	finally:
		ackermannDriveMsg = AckermannDriveMsg()
		#twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		#twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(AckermannDriveMsg)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


