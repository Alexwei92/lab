#!/usr/bin/env python

# Simulate different modes with Joysticks
# Author: Peng Wei
# Last Update: 10/04/2018

import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Twist
from lab.msg import Joyfilter
from enum import Enum

# Global parameters
MAX_THROTTLE = 60000
MIN_THROTTLE = 35000
ARM_THROTTLE = 20000

MAX_ROLL 	= 30.0
MAX_PITCH 	= 30.0
MAX_YAWRATE = 120.0
MAX_THRUST 	= 0.80

# Enum MODE
class MODE(Enum):
	STABLIZE = 1
	LOITER 	 = 2
	ALTHOLD  = 3
	POSHOLD  = 4
	RTL 	 = 5
	AUTO 	 = 6

# Main class
class Joystick():
	def __init__(self, joy_topic):

		# Initialize the service
		#rospy.wait_for_service('emergency')
		self._emergency = rospy.ServiceProxy('emergency', Empty)

		# Initialize the parameters
		self.joy = Joyfilter()
		self.twist = Twist()
		self._mode = MODE.STABLIZE    # STABLIZE by default
		#self.msg = PoseStamped()
		#self.msg.header.seq = 0
		#self.msg.header.stamp = rospy.Time.now()
		self._buttons = None
		

		rospy.Subscriber(joy_topic, Joy, self._joyChanged)

	# Read the mode
	def get_mode(self):
		return self._mode.name

	# Set the mode
	def set_mode(self, value):
		self._mode = Color[value]
		rospy.loginfo(value + ' Mode')

	# A low-pass filter on joy axis
	def LP_filter(self, data):
		a1 = 0.8
		a2 = 0.8
		a3 = 0.8
		a4 = 1.0
		self.joy.axes0 = a1*data.axes[0] + (1-a1)*self.joy.axes0
		self.joy.axes1 = a2*data.axes[1] + (1-a2)*self.joy.axes1
		self.joy.axes2 = a3*data.axes[2] + (1-a3)*self.joy.axes2
		self.joy.axes3 = a4*data.axes[3] + (1-a4)*self.joy.axes3
		return self.joy	

	# A linear mapping	
	def linear_map(self, x, old_min, old_max, new_min, new_max):
		return (x-old_min)/(old_max-old_min)*(new_max-new_min)+new_min

	# check the button pressed
	def _joyChanged(self, data):
		self.LP_filter(data)
		for i in range(0, len(data.buttons)):
			if self._buttons == None or data.buttons[i] != self._buttons[i]:
				#if i == 0 and data.buttons[i] == 1:
					#self._land()
					#rospy.loginfo("Landing requested!")
				if i == 1 and data.buttons[i] == 1:
					self._emergency()
					rospy.loginfo("Emergency requested!")
				#if i == 2 and data.buttons[i] == 1:
					#self._takeoff()
					#rospy.loginfo("TakeOff requested!")
				if i == 3 and data.buttons[i] == 1:
					self.set_mode('STABLIZE')
				if i == 4 and data.buttons[i] == 1:
					self.set_mode('LOITER')
				if i == 5 and data.buttons[i] == 1:
					self.set_mode('ALTHOLD')
				if i == 6 and data.buttons[i] == 1:
					self.set_mode('POSHOLD')
				if i == 7 and data.buttons[i] == 1:
					self.set_mode('RTL')
				if i == 8 and data.buttons[i] == 1:
					self.set_mode('AUTO')
		self._buttons = data.buttons

	# decide the current mode
	def _decideMode(self, ):
		if self.get_mode() == 'STABLIZE':
			self._stablize_mapping()	

		elif self.get_mode() == 'LOITER':
			self._loiter_mapping()
		
		elif self.get_mode() == 'ALTHOLD':
			self._althold_mapping()
		
		elif self.get_mode() == 'POSHOLD':
			self._poshold_mapping()
		
		elif self.get_mode() == 'RTL':
			self._rtl_mapping()
		
		elif self.get_mode() == 'AUTO':
			self._auto_mapping()

	# Stablize mode mapping
	def _stablize_mapping(self):
		# roll, pitch, yaw rate mapping
		self.twist.linear.y  = -1*self.linear_map(self.joy.axes0,-1, 1, -MAX_ROLL, MAX_ROLL) 
		self.twist.linear.x  =  1*self.linear_map(self.joy.axes1,-1, 1, -MAX_PITCH, MAX_PITCH)
		self.twist.angular.z = -1*self.linear_map(self.joy.axes2,-1, 1, -MAX_YAWRATE, MAX_YAWRATE) 
		# throttle mapping
		deadzone = -0.8
		if self.joy.axes3 < deadzone: #throttle deadzone
			self.twist.linear.z = ARM_THROTTLE
		else:
			self.twist.linear.z = self.linear_map(self.joy.axes3, deadzone, 1, MIN_THROTTLE, MAX_THROTTLE*MAX_THRUST)

	# Execute the file
	def execute(self):
		self._decideMode()
		if self.get_mode() in {'STABLIZE', 'LOITER'}:
			return self.twist



if __name__ == '__main__':
	rospy.init_node('joystick', anonymous=True)
	joy_topic = rospy.get_param("~joy_topic", "/joy")
	target = rospy.get_param("~target", "/goal")
	r = rospy.get_param("~rate", 50)
	rate = rospy.Rate(r)

	Sim_Mode = Joystick(joy_topic)  
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

	while not rospy.is_shutdown():
		pub.publish(Sim_Mode.execute())
		rate.sleep