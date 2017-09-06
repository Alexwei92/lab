#!/usr/bin/env python
# Formation of three drones
#
#       *
#      - -
#     * - *
#
# Author: Peng Wei

import rospy
import tf
import math

from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

import math

class Joystick():
	def __init__(self):
		self.x_leader 	 = 0.0
		self.y_leader 	 = 0.0
		self.z_leader 	 = 1.0
		self.yaw_leader  = 0.0


		self.r1 = [1.0, 0.0, 0.0]
		self.r2 = [1.0, 0.0, 0.0]
		self.r3 = [0.0, 1.0, 0.0]
		
		self.phi = 0.0
		self.theta = 0.0
		self.psi = 0.0
		
		self.msg = Pose()
    		self.start_time = rospy.Time.now()
	    	self.msg.pose.position.x = self.x_leader
	    	self.msg.pose.position.y = self.y_leader
	    	self.msg.pose.position.z = self.z_leader
	    	quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
	    	self.msg.pose.orientation.x = quaternion[0]
	    	self.msg.pose.orientation.y = quaternion[1]
	    	self.msg.pose.orientation.z = quaternion[2]
	    	self.msg.pose.orientation.w = quaternion[3]
	
		self.msg1 = PoseStamped()
    		self.start_time = rospy.Time.now()
	    	self.msg.pose.position.x = self.x_leader+rotation()
	    	self.msg.pose.position.y = self.y_leader
	    	self.msg.pose.position.z = self.z_leader
	    	quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
	    	self.msg.pose.orientation.x = quaternion[0]
	    	self.msg.pose.orientation.y = quaternion[1]
	    	self.msg.pose.orientation.z = quaternion[2]
	    	self.msg.pose.orientation.w = quaternion[3]

	def rotation(self, source, phi, theta, psi):
		sp    = math.sin(phi);
		cp    = math.cos(phi);
		st    = math.sin(theta);
		ct    = math.cos(theta);
	    	ss    = math.sin(psi);
	    	cs    = math.cos(psi);
		target = [0.0]*3
		target[0] = ct*cs*source[0] + (sp*st*cs-cp*ss)*source[1] +(cp*st*cs+sp*ss)*source[2];
	    	target[1] = ct*ss*source[0] + (sp*st*ss+cp*cs)*source[1] + (cp*st*ss-sp*cs)*source[2];
	    	target[2] = -st*source[0] + sp*ct*source[1] + cp*ct*source[2];
		return target
	
	def linear_map(self, x, r1_min, r1_max, r2_min, r2_max):
		return (x-r1_min)/(r1_max-r1_min)*(r2_max-r2_min)+r2_min

	def update(self):
		current_time = rospy.Time.now()
		dt = current_time.to_sec() - self.start_time.to_sec();

		self.msg.pose.position.x = self.radius*math.cos(0.4*math.pi*dt) + self.x_origin
		self.msg.pose.position.y = self.radius*math.sin(0.4*math.pi*dt) + self.y_origin
		self.msg.pose.position.z = 0.5
		# rospy.loginfo("Time: %f, x = %f, y= %f, z= %f", dt, self.msg.pose.position.x, self.msg.pose.position.y,self.msg.pose.position.z)
		# yaw =  0.0;
		# quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		# self.msg.pose.orientation.x = quaternion[0]
		# self.msg.pose.orientation.y = quaternion[1]
		# self.msg.pose.orientation.z = quaternion[2]
		# self.msg.pose.orientation.w = quaternion[3]
		return self.msg

if __name__ == '__main__':
	rospy.init_node('formation')
	name = rospy.get_param("~name", 'goal')     # publish to
	r = rospy.get_param("~rate","50.0")	    # frequency 
	rate = rospy.Rate(50)

	goal = Joystick()
	rospy.loginfo("Start waypoint control")
	pub = rospy.Publisher(name, PoseStamped, queue_size=20)

	while not rospy.is_shutdown():
		goal.msg.header.seq += 1
		goal.msg.header.stamp = rospy.Time.now()
		pub.publish(goal.update())
		rate.sleep()
