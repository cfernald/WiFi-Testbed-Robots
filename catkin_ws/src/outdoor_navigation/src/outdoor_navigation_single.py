#!/usr/bin/env python
# Last Edited by Siavash Zangeneh, 8.21.2014

# GPS Navigation node, subsribes to GPS Measurement and Compass Measurement topics and publishes Traxxas Ackermann topic


import sys
import math

import rospy, os, pwd
from std_msgs.msg import String
from traxxas_node.msg import AckermannDriveMsg
from proteus3_gps_hydro.msg import GPSMsg
from proteus3_compass_hydro.msg import CompassMsg


# variables, you can adjust these variables for calibratyion
close_enough = 4  # distance in meters considered close enough to destination to stop
move_speed = 1    # desired move speed of the traxxas

#Controller parameters,
Kp = 0.1	#proportional gain, Should be non-zero (Tested with 0.03125)
Ki = 0.01	#integral gain, should be non-negative (Tested with 0.03)
Kd = 0		#Differential gain, should be non-negative (Tested with 0)
Kl = 0		#Divergence to the line gain, should be non-negative (Tested with 0)
max_total_error = 400 #maximum integral sum of error


# DO NOT CHANGE ANY GLOBAL VARIABLES DEFINED BELOW

# Initialize GPS and compass readings
cur_lat = 0
cur_lon = 0
heading = 0
heading_gps = 0
pitch = 0
roll = 0

# Global Waypoint variables
num_waypoints = 0
waypoints = []
waypoint_index = 0
LineToWaypoint = None
dest_lat = 0
dest_lon = 0

# a class abstracting lines
class Line:
	#Public method, updates all line parameters based on start and end coordinates of the line
	def update(self, startlat, startlon, endlat, endlon):
		self.start_lat = startlat
		self.start_lon = startlon
		self.end_lat = endlat
		self.end_lon = endlon
		self.heading_angle = get_angle(startlat, startlon, endlat, endlon)
		if (startlon == endlon):
			self.isVertical = True
			self.xIntersect = startlon
		else:
			self.isVertical = False
			self.slope = (startlat - endlat) / (startlon - endlon)
			self.yIntersect = startlat - self.slope * startlon

	def __init__(self):
		self.update( 0,0,0,0)
	def __init__(self, startlat, startlon, endlat, endlon):
		self.update(startlat, startlon, endlat, endlon)
	
	#Public Method, returns the distance of the line to the location passed as argument
	def findDistanceToLocation(self, loclat, loclon):
		#finding the closest point on the line to the location
		if (self.isVertical == True):
			closest_lat = loclat
			closest_lon = self.xIntersect	
		elif (self.slope == 0):
			closest_lat = self.yIntersect
			closest_lon = loclon
		else:
			#finding the perpendicular line to the line
			pSlope = -1 * (1/self.slope)	
			pYIntersect = loclat - pSlope * loclon
			#find the intersection of the original line and the perpendicular line passing through the point of interest
			closest_lon = ( pYIntersect - self.yIntersect ) / ( self.slope - pSlope)
			closest_lat = self.slope * closest_lon + self.yIntersect
		#find the heading difference of the line and the location (find whether the location is on left or right of the line)
		ang_difference = self.heading_angle - get_angle(self.start_lat, self.start_lon, loclat, loclon)	
		#return the distance to the closest point, positive if right, negative if left 
		if (ang_difference>= 0):
			return get_distance(loclat, loclon, closest_lat, closest_lon)
		else:
			return -1*get_distance(loclat, loclon, closest_lat, closest_lon)



# adds the GPS coordinates found in a file to the waypoints variable
# Arguments:
# filename: Absolute address of the file containing the waypoints
# returns nothing
# Description: file should contain the coordinates of each waypoint on a separate line. Latitude and longitude should be separated by a comma.
# Each line should contain two or more comma-separated elements. (All elements except the first two are ignored)
# If there are not two elements or each line, or the elements cannot be converted to floats, the function will fail and stop the node
def initialize_waypoints(filename):
	global waypoints
	global num_waypoints
	global dest_lat
	global dest_lon
	f = open(filename)
	for line in f:
		waypoints.append([])
                temp_list = line.split(',')
		waypoints[num_waypoints].append(float(temp_list[0]))	#crashes the node if the elements does not exist / is not a number
		waypoints[num_waypoints].append(float(temp_list[1]))	#crashes the node if the elements does not exist / is not a number
		num_waypoints += 1
	dest_lat = waypoints[waypoint_index][0]
	dest_lon = waypoints[waypoint_index][1]

	print "File opened successfully: ", filename
	print waypoints

# gets robot's current longitude and latitude from gps msg
def get_gps(data):
	global cur_lat
	global cur_lon
	global heading_gps
	global LineToWaypoint
	cur_lat = data.latitude
	cur_lon = data.longitude
	heading_gps = data.heading
	if (LineToWaypoint is None):
		LineToWaypoint = Line(cur_lat, cur_lon, waypoints[waypoint_index][0], waypoints[waypoint_index][1])
 	#print rospy.get_caller_id(),'Received Coordinates - Latitude: ',data.latitude,' Longitude: ',data.longitude


# Global variables required for the compass average filter
FILTER_SIZE = 5
filter_buffer = [0] * FILTER_SIZE 
filter_index = 0
filter_sum = 0
FILTER_ON = False #Change this to activate or deactivate the filter
# gets robot's current heading, pitch, and roll, from compass msg
def get_compass(data):
	global heading
	global pitch
	global roll
	global filter_buffer
	global filter_index
	global filter_sum
	
	#update the buffer to keep track of the last readings of the compass
	filter_buffer[filter_index] = data.heading
	filter_sum = 0
	#Sum the angle difference of all the readings in the buffer to the new reading
	for datapoint in filter_buffer:
		filter_sum += get_heading_error(filter_buffer[filter_index], datapoint)
	
	if (FILTER_ON == True):
		#Average filter result = The new reading + the average of the difference of the old readings to the new reading
		heading = filter_buffer[filter_index] + filter_sum/FILTER_SIZE
	else:
		heading = data.heading

	# normalize the heading after the filter addition
	if heading > 180:
		heading -= 360
	elif heading <= -180:
		heading += 360

	#update the filter index which points to the oldest data in the filter
	filter_index += 1
	if filter_index >= FILTER_SIZE:
		filter_index = 0
		
	#update pitch and roll variables, currently unused in the navigation
	pitch = data.pitch
	roll = data.roll
	#update ackermann commands with each new compass readings
	update_navigation()	


	#print rospy.get_caller_id(),'Received Compass Info - Heading: ',data.heading


# Debug tool, call to print basic information
def check():
	print 'Current Location:', cur_lat, cur_lon
	print 'Destination:', dest_lat, dest_lon

	distance = get_distance(cur_lat, cur_lon, dest_lat, dest_lon)

	print "Distance to destination:", distance

# calculates distance from location A (lon_A, lat_A) to B (lon_B, lat_B)
def get_distance(lat_A, lon_A, lat_B, lon_B):
	# uses "Haversine Formula"

	# approx radius of earth in kilometers
	R = 6371 

	# calculations
	lon_D = math.radians(lon_B - lon_A)
	lat_D = math.radians(lat_B - lat_A)

	lat_A = math.radians(lat_A)
	lat_B = math.radians(lat_B)

	a = ( math.sin(lat_D / 2) * math.sin(lat_D / 2)
		 + math.sin(lon_D / 2) * math.sin(lon_D / 2)
		 * math.cos(lat_A) * math.cos(lat_B) )

	c = 2 * math.atan2( math.sqrt(a) , math.sqrt(1-a) )

	d = R * c;

	d = d *1000  # Return meters

	return d

#Controller parameters,
total_error = 0		#should be 0
previous_error = 0	#should be 0

def update_navigation():
	global total_error
	global previous_error
	global dest_lat
	global dest_lon
	global waypoint_index
	if (LineToWaypoint is None):
		print "No GPS Signal Yet ... "
		return False

	distance = get_distance(cur_lat, cur_lon, dest_lat, dest_lon)

	#If arrived at the next waypoint, update the waypoint to the next, start with first if reached to last waypoint
	if distance < close_enough:
		waypoint_index += 1
		if waypoint_index >= num_waypoints:
			waypoint_index = 0
		dest_lat = waypoints[waypoint_index][0]
		dest_lon = waypoints[waypoint_index][1]
		distance = get_distance(cur_lat, cur_lon, dest_lat, dest_lon)
		LineToWaypoint.update(cur_lat, cur_lon, dest_lat, dest_lon)


	print 'Navigating to waypoint ', waypoint_index
	print 'Distance to Destination', distance

	angle_to_target = get_angle(cur_lat, cur_lon, dest_lat, dest_lon)

	print 'Angle to target = ', angle_to_target

	heading_error = get_heading_error(heading, angle_to_target)

	print 'Heading error = ', heading_error

	print 'Received Compass Info - Heading: ', heading
	print 'Received GPS     Info - Heading: ', heading_gps
 	print 'Received Coordinates - Latitude: ',cur_lat,' Longitude: ',cur_lon
	print 'Destination - Latitude: ', dest_lat, ' Longitude: ', dest_lon


	#Updates the integral term of the PID
	total_error += heading_error
	if total_error > max_total_error:
		total_error = max_total_error
	elif total_error < -max_total_error:
		total_error = -max_total_error

	#Updates the differential term fo the PID
	differential_error = heading_error - previous_error
	previous_error = heading_error

	#Updates the distance to the line
	divergence = LineToWaypoint.findDistanceToLocation(cur_lat, cur_lon)

	print 'Divergence from line', divergence

	# set steering angle
	# max_steering for traxxas is 0.35 radians (20 degrees)
	# Updates the steering command using all 4 calculated terms. To edit the coefficents, update the variables defined above the function 	
	steering = Kp * heading_error + Ki * total_error + Kd * differential_error + Kl * divergence

	#calculate the appropriate speed based on the distance
	speed = calculate_speed(distance,move_speed,heading_error)

	#Make sure the steering command does not go out of range	
	max_steering = 20
	if steering > max_steering:
		steering = max_steering
	elif steering < - max_steering:
		steering = - max_steering
	else:
		steering = heading_error


	print 'Steering = ', steering, 'Speed = ', speed
	# Publish the traxxas commands
	move(speed, steering)

	return False


# Calculates the appropriate speed based on the desired steering and distance to the destination
# minimum speed is 0.5
def calculate_speed(distance, desired_speed, desired_steering):

	if (math.fabs(desired_steering) > 120):
		max_speed = 0.6
	elif (distance > 6):
		max_speed = 4
	elif (distance > 5):
		max_speed = 1.5
	elif (distance > 4):
		max_speed = 1
	elif (distance > 3):
		max_speed = 0.7
	else:
		max_speed = 0.5

	if (desired_speed <= max_speed):
		return desired_speed
	else:
		return max_speed


# Calculates the heading angle based on the current coordinates and the destination coordinates
def get_angle(cur_lat, cur_lon, dest_lat, dest_lon):
	#TODO Handle vertical, horizontal

	x_err = dest_lat - cur_lat
	y_err = -1 * (dest_lon - cur_lon)

	angle_radians = math.atan2(y_err, x_err)

	angle_deg = math.degrees(angle_radians)

	return angle_deg

# Calculates the difference between two angles and acounts for periodic behavior of the angle circle
def get_heading_error (angle1, angle2):
	difference = angle2 - angle1
	if difference > 180:
		difference -= 360
	elif difference <= -180:
		difference += 360
	return difference


def move(speed, steering):
	# publish speed and steering to traxxas_node AckermannDriveMsg
	ackermann_drive_msg = AckermannDriveMsg()
	ackermann_drive_msg.speed = speed
	ackermann_drive_msg.steering_angle = steering
	pub.publish(ackermann_drive_msg)




if __name__ == '__main__':
	rospy.init_node('outdoor_navigation', anonymous=True)
	#argv[2] is the start waypoint index
	waypoint_index = int(sys.argv[2])
	#argv[1] is the name of the file which should be saved in ~/ros_outdoor_navigation_data/
	initialize_waypoints('/home/'+pwd.getpwuid(os.getuid())[0]+'/ros_outdoor_navigation_data/'+sys.argv[1])
	
	rospy.Subscriber("gps/measurement", GPSMsg, get_gps)

	rospy.Subscriber("compass/measurement", CompassMsg, get_compass) 
	pub = rospy.Publisher('traxxas_node/ackermann_drive', AckermannDriveMsg)

	rospy.spin()
