#!/usr/bin/env python
# Joystick control with a hovering point
# Author: Peng Wei

import rospy
import tf
import math

from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from lab.msg import Joyfilter

class Joystick():
	def __init__(self, x_origin, y_origin, x_max, y_max, z_max, yaw_max):
		self.x_origin = x_origin
		self.y_origin = y_origin
		self.x_max    = x_max
		self.y_max    = y_max
		self.z_max    = z_max
		self.yaw_max  = yaw_max
		self.joy = Joyfilter()
		self.joy.axes0 = -0.0
		self.joy.axes1 = -0.0
		self.joy.axes2 = -0.0
		self.joy.axes3 = -1.0

		self.msg = PoseStamped()
		self.msg.header.seq = 0
    		self.msg.header.stamp = rospy.Time.now()
    		#self.start_time = self.msg.header.stamp
	    	self.msg.header.frame_id = "/hover"
	    	self.msg.pose.position.x = self.x_origin
	    	self.msg.pose.position.y = self.y_origin
	    	self.msg.pose.position.z = self.z_max
	    	quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
	    	self.msg.pose.orientation.x = quaternion[0]
	    	self.msg.pose.orientation.y = quaternion[1]
	    	self.msg.pose.orientation.z = quaternion[2]
	    	self.msg.pose.orientation.w = quaternion[3]

	def linear_map(self, x, r1_min, r1_max, r2_min, r2_max):
		output = (x-r1_min)/(r1_max-r1_min)*(r2_max-r2_min)+r2_min
		if abs(output)<1e-3:
			output = 0.0
		return output

	def filter_joy(self, data):
		a = 0.2
		self.joy.axes0 = a*data.axes[0] + (1-a)*self.joy.axes0
		self.joy.axes1 = a*data.axes[1] + (1-a)*self.joy.axes1
		self.joy.axes2 = a*data.axes[2] + (1-a)*self.joy.axes2
		self.joy.axes3 = a*data.axes[3] + (1-a)*self.joy.axes3
		return self.joy		


	def mapping(self, data):
		joy = self.filter_joy(data)
		#self.msg.pose.position.x = self.linear_map(-data.axes[0],-1, 1, self.x_origin-self.x_max, self.x_origin+self.x_max)
		#self.msg.pose.position.y = self.linear_map( data.axes[1],-1, 1, self.y_origin-self.y_max, self.y_origin+self.y_max)
		#self.msg.pose.position.z = self.linear_map( data.axes[3],-1, 1, 0, self.z_max)
		#yaw =  self.linear_map(data.axes[2],-1, 1, -math.radians(self.yaw_max), math.radians(self.yaw_max))
		
		self.msg.pose.position.x = self.linear_map(-joy.axes0,-1, 1, self.x_origin-self.x_max, self.x_origin+self.x_max)
		self.msg.pose.position.y = self.linear_map( joy.axes1,-1, 1, self.y_origin-self.y_max, self.y_origin+self.y_max)
		self.msg.pose.position.z = self.linear_map( joy.axes3,-1, 1, 0, self.z_max)
		yaw =  self.linear_map(joy.axes2,-1, 1, -math.radians(self.yaw_max), math.radians(self.yaw_max))
		
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		self.msg.pose.orientation.x = quaternion[0]
		self.msg.pose.orientation.y = quaternion[1]
		self.msg.pose.orientation.z = quaternion[2]
		self.msg.pose.orientation.w = quaternion[3]


if __name__ == '__main__':
	rospy.init_node('joystick_hover')
	name = rospy.get_param("~name", '/goal') 
	r = rospy.get_param("~rate", 50)	 
	rate = rospy.Rate(r)
	x_origin = rospy.get_param("~x_origin", -1.0)
	y_origin = rospy.get_param("~y_origin", 0.0)
	x_max = rospy.get_param("~x_max", 0.5)
	y_max = rospy.get_param("~y_max", 0.5)
	z_max = rospy.get_param("~z_max", 1.0)
	yaw_max = rospy.get_param("~yaw_max", 120)

	goal = Joystick(x_origin, y_origin, x_max, y_max, z_max, yaw_max)
	rospy.loginfo("Mapping the joystick input")
	rospy.Subscriber('/joy', Joy, goal.mapping)
	pub = rospy.Publisher(name, PoseStamped, queue_size=5)

	while not rospy.is_shutdown():
		goal.msg.header.seq += 1
		goal.msg.header.stamp = rospy.Time.now()
		pub.publish(goal.msg)
		rate.sleep()
