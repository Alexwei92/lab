#!/usr/bin/env python
# J
# Author: Peng Wei

import rospy
import tf

from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

import math

class Joystick():
	def __init__(self):
		self.x_origin = -1.0
		self.y_origin = 0.0
		self.radius   = 1.0

		self.z_max    = 1.0
		self.yaw      = 0.0

		self.msg = PoseStamped()
		self.msg.header.seq = 0
    		self.msg.header.stamp = rospy.Time.now()
    		self.start_time = rospy.Time()
    		self.start_time = self.msg.header.stamp
	    	self.msg.header.frame_id = "/world"
	    	self.msg.pose.position.x = self.x_origin
	    	self.msg.pose.position.y = self.y_origin
	    	self.msg.pose.position.z = self.z_max
	    	quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
	    	self.msg.pose.orientation.x = quaternion[0]
	    	self.msg.pose.orientation.y = quaternion[1]
	    	self.msg.pose.orientation.z = quaternion[2]
	    	self.msg.pose.orientation.w = quaternion[3]

	def linear_map(self, x, r1_min, r1_max, r2_min, r2_max):
		return (x-r1_min)/(r1_max-r1_min)*(r2_max-r2_min)+r2_min

	def update(self, data):
		current_time = rospy.Time()
		current_time - rospy.Time.now()
		dt = current_time.to_sec() - self.start_time.to_sec();
		rospy.loginfo("goal time: %f", current_time - self.start_time.to_sec())
		self.msg.pose.position.x = self.radius*cos(0.02*pi*dt)
		self.msg.pose.position.y = self.radius*sin(0.02*pi*dt)
		self.msg.pose.position.z = self.linear_map( data.axes[3],-1, 1, 0, self.z_max)

		# yaw =  0.0;
		# quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		# self.msg.pose.orientation.x = quaternion[0]
		# self.msg.pose.orientation.y = quaternion[1]
		# self.msg.pose.orientation.z = quaternion[2]
		# self.msg.pose.orientation.w = quaternion[3]


if __name__ == '__main__':
	rospy.init_node('joystick_hover')
	name = rospy.get_param("~name", 'goal')     # publish to
	r = rospy.get_param("~rate","50.0")	    # frequency 
	rate = rospy.Rate(50)

	goal = Joystick()
	rospy.loginfo("Start waypoint control")
	rospy.Subscriber('/joy', Joy, goal.update)
	pub = rospy.Publisher(name, PoseStamped, queue_size=5)

	while not rospy.is_shutdown():
		goal.msg.header.seq += 1
		goal.msg.header.stamp = rospy.Time.now()
		pub.publish(goal.msg)
		rate.sleep()
