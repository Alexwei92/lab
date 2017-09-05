#!/usr/bin/env python
# Mapping joystick output to /crazyflie/cmd_vel
# Author: Peng Wei

import rospy

from crazyflie_driver.srv import UpdateParams
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

MAX_THROTTLE = 65535
MIN_THROTTLE = 10000

class Joystick():
	def __init__(self, Max_roll, Max_pitch, Max_yaw, Max_thrust, Min_thrust):
		self.Max_roll = Max_roll
		self.Max_pitch = Max_pitch
		self.Max_yaw = Max_yaw
		self.Max_thrust = Max_thrust
		self.Min_thrust = Min_thrust
		self.twist = Twist()
		self.i = 0

	def linear_map(self, x, r1_min, r1_max, r2_min, r2_max):
		return (x-r1_min)/(r1_max-r1_min)*(r2_max-r2_min)+r2_min

	def mapping(self, data):
		# roll, pitch, yaw mapping
		self.twist.linear.y  = -1*self.linear_map(data.axes[0],-1, 1, -self.Max_roll, self.Max_roll) 
		self.twist.linear.x  =  1*self.linear_map(data.axes[1],-1, 1, -self.Max_pitch, self.Max_pitch)
		self.twist.angular.z = -1*self.linear_map(data.axes[2],-1, 1, -self.Max_yaw, self.Max_yaw) 
		# thrust mapping
		if data.axes[3] < -0.80:
			self.twist.linear.z = MIN_THROTTLE
		else:
			self.twist.linear.z = self.linear_map(data.axes[3],-1, 1, MAX_THROTTLE*Min_thrust, MAX_THROTTLE*Max_thrust)


if __name__ == '__main__':
	rospy.init_node('joystick_control')
	Max_roll   = rospy.get_param("~Max_roll_angle", 30.0)
	Max_pitch  = rospy.get_param("~Max_pitch_angle", 30.0)
	Max_yaw    = rospy.get_param("~Max_yaw_rate", 180.0)
	Max_thrust = rospy.get_param("~Max_thrust", 0.80)
	Min_thrust = rospy.get_param("~Min_thrust", 0.25)

	joystick = Joystick(Max_roll, Max_pitch, Max_yaw, Max_thrust, Min_thrust)
	rospy.loginfo("Mapping the joystick input")
	rospy.Subscriber('/joy', Joy, joystick.mapping)
	pub = rospy.Publisher('/crazyflie/cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		pub.publish(joystick.twist)
		rate.sleep()

