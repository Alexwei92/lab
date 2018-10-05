#!/usr/bin/env python
# Mapping joystick output to /crazyflie/cmd_vel
# Author: Peng Wei

import rospy

#from crazyflie_driver.srv import UpdateParams
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from lab.msg import Joyfilter

MAX_THROTTLE = 60000
MIN_THROTTLE = 35000
ARM_THROTTLE = 20000

class Joystick():
	def __init__(self, Max_roll, Max_pitch, Max_yaw, Max_thrust):
		self.Max_roll = Max_roll
		self.Max_pitch = Max_pitch
		self.Max_yaw = Max_yaw
		self.Max_thrust = Max_thrust
		self.twist = Twist()
		self.joy = Joyfilter()
		self.joy.axes0 = -0.0
		self.joy.axes1 = -0.0
		self.joy.axes2 = -0.0
		self.joy.axes3 = -1.0

	def filter_joy(self, data):
		a = 0.6
		self.joy.axes0 = a*data.axes[0] + (1-a)*self.joy.axes0
		self.joy.axes1 = a*data.axes[1] + (1-a)*self.joy.axes1
		self.joy.axes2 = a*data.axes[2] + (1-a)*self.joy.axes2
		self.joy.axes3 = a*data.axes[3] + (1-a)*self.joy.axes3
		return self.joy	

	def linear_map(self, x, r1_min, r1_max, r2_min, r2_max):
		return (x-r1_min)/(r1_max-r1_min)*(r2_max-r2_min)+r2_min

	def mapping(self, data):
		# roll, pitch, yaw rate mapping
		self.filter_joy(data)
		self.twist.linear.y  = -1*self.linear_map(self.joy.axes0,-1, 1, -self.Max_roll, self.Max_roll) 
		self.twist.linear.x  =  1*self.linear_map(self.joy.axes1,-1, 1, -self.Max_pitch, self.Max_pitch)
		self.twist.angular.z = -1*self.linear_map(self.joy.axes2,-1, 1, -self.Max_yaw, self.Max_yaw) 
		# thrust mapping
		if self.joy.axes3 < -0.8:
			self.twist.linear.z = ARM_THROTTLE
		else:
			self.twist.linear.z = self.linear_map(self.joy.axes3,-0.8, 1, MIN_THROTTLE, MAX_THROTTLE*self.Max_thrust)

if __name__ == '__main__':
	rospy.init_node('pose')
	name = rospy.get_param("~joy_topic", '/joy')
	Max_roll   = rospy.get_param("~Max_roll_angle", 30.0)
	Max_pitch  = rospy.get_param("~Max_pitch_angle", 30.0)
	Max_yaw    = rospy.get_param("~Max_yaw_rate", 120.0)
	Max_thrust = rospy.get_param("~Max_thrust", 0.80)
	#Min_thrust = rospy.get_param("~Min_thrust", 0.25)

	joystick = Joystick(Max_roll, Max_pitch, Max_yaw, Max_thrust)
	rospy.loginfo("Arming!")
	rospy.sleep(1.0)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
	rate = rospy.Rate(50)
	rospy.Subscriber(name, Joy, joystick.mapping)
	while not rospy.is_shutdown():
		pub.publish(joystick.twist)
		rate.sleep()