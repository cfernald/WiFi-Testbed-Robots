#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_ackermann_keyboard')
import rospy

from traxxas_node.msg import AckermannDriveMsg
from proteus3_gps_hydro.msg import GPSMsg

import os
import sys, select, termios, tty

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

cur_lat = 30.3423
cur_lon = -97.43242342
def get_gps(data):
        global cur_lat
        global cur_lon
        cur_lat = data.latitude
        cur_lon = data.longitude

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	if (len(sys.argv) < 2):
		print 'Pass the waypoint file name as an argument'
	else: 
		filename = '/home/'+os.getlogin()+'/ros_outdoor_navigation_data/'+sys.argv[1]
		rospy.init_node('create_navigation_waypoint')
		rospy.Subscriber("gps/measurement", GPSMsg, get_gps)
		f = open(filename, 'w')
		stopped = False
		i = 0
		while(not stopped):
			while True:
				key = getKey()
				print key
				if (key == ' '):
					break
				if (key == '\x03'):  # end of text char 
					stopped = True
					break
			if (stopped == True):
				break

			print 'Waypoint '+str(i)+' created!'
			i += 1
			f.write(str(cur_lat)+','+str(cur_lon)+'\n');
			rospy.sleep(3)

