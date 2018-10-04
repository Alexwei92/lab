#!/usr/bin/env python
# Fly in a circular pattern
# Author: Peng Wei

import rospy
import tf
import math

from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

class Joystick():
	def __init__(self, x_origin, y_origin, z_origin, z_max, radius):
		self.x_origin = x_origin
		self.y_origin = y_origin
		self.z_origin = z_origin
		self.z_max    = z_max
		self.radius   = radius
		
		self.msg = PoseStamped()
		self.msg.header.seq = 0
		self.msg.header.stamp = rospy.Time.now()
		self.start_time = self.msg.header.stamp
		self.msg.header.frame_id = "/world"
		self.msg.pose.position.x = self.x_origin
		self.msg.pose.position.y = self.y_origin
		self.msg.pose.position.z = self.z_max
		quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
		self.msg.pose.orientation.x = quaternion[0]
		self.msg.pose.orientation.y = quaternion[1]
		self.msg.pose.orientation.z = quaternion[2]
		self.msg.pose.orientation.w = quaternion[3]

	def linear_map(self, x, r1_min, r1_max, r2_min, r2_max):
		return (x-r1_min)/(r1_max-r1_min)*(r2_max-r2_min)+r2_min

	def update(self):
		current_time = rospy.Time.now()
		dt = current_time.to_sec() - self.start_time.to_sec();
		
		# adjust the during
		t1 = 10.0
		t2 = 40.0
		t3 = 45.0
		t4 = 75.0
		w1 = 4*math.pi/(t2-t1)
		w2 = 4*math.pi/(t4-t3)
		w3 = math.pi/(t4-t3)

		if (dt > t1 and dt <= t2):
			self.msg.pose.position.x = self.radius*math.cos(w1*(dt-t1)) + self.x_origin
			self.msg.pose.position.y = self.radius*math.sin(w1*(dt-t1)) + self.y_origin
			self.msg.pose.position.z = self.z_origin
		elif (dt > t3 and dt <= t4):
			self.msg.pose.position.x = self.radius*math.cos(w2*(dt-t3)) + self.x_origin
			self.msg.pose.position.y = self.radius*math.sin(w2*(dt-t3)) + self.y_origin
			self.msg.pose.position.z = self.z_origin + (self.z_max-self.z_origin)*math.sin(w3*(dt-t3))
		else :		
			self.msg.pose.position.x = self.radius*math.cos(0.0) + self.x_origin
			self.msg.pose.position.y = self.radius*math.sin(0.0) + self.y_origin
			self.msg.pose.position.z = self.z_origin

		return self.msg

if __name__ == '__main__':
	rospy.init_node('pose')
	name = rospy.get_param("~name", '/goal')
	x_origin = rospy.get_param("~x_origin", 0.0)
	y_origin = rospy.get_param("~y_origin", 0.0)
	z_origin = rospy.get_param("~z_origin", 0.7)
	z_max = rospy.get_param("~z_max", 1.5)
	radius = rospy.get_param("~radius", 1.0)

	r = rospy.get_param("~rate", 30)
	rate = rospy.Rate(r)

	goal = Joystick(x_origin, y_origin, z_origin, z_max, radius)
	#rospy.loginfo("Start waypoint control")
	pub = rospy.Publisher(name, PoseStamped, queue_size=10)

	while not rospy.is_shutdown():
		goal.msg.header.seq += 1
		goal.msg.header.stamp = rospy.Time.now()
		pub.publish(goal.update())
		rate.sleep()
