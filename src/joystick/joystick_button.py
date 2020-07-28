#!/usr/bin/env python
# Joystick buttons mapping
# Author: Peng Wei
# Last Update: 10/03/2018

import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

class Controller():
	def __init__(self, joy_topic):
		rospy.wait_for_service('emergency')
		self._emergency = rospy.ServiceProxy('emergency', Empty)

		#rospy.wait_for_service('land')
		self._land = rospy.ServiceProxy('landing', Empty)
		#rospy.wait_for_service('takeoff')
		self._takeoff = rospy.ServiceProxy('take_off', Empty)

		self._update = rospy.ServiceProxy('update', Empty)

		# subscribe to the joystick at the end to make sure that all required
		# services were found
		self._buttons = None
		rospy.Subscriber(joy_topic, Joy, self._joyChanged)

	def _joyChanged(self, data):
		for i in range(0, len(data.buttons)):
			if self._buttons == None or data.buttons[i] != self._buttons[i]:
				if i == 0 and data.buttons[i] == 1:
					self._land()
					#rospy.loginfo("Landing requested!")
				if i == 1 and data.buttons[i] == 1:
					self._emergency()
					#rospy.loginfo("Emergency requested!")
				if i == 2 and data.buttons[i] == 1:
					self._takeoff()
					#rospy.loginfo("TakeOff requested!")
				if i == 6 and data.buttons[i] == 1:
					self._update()
					rospy.loginfo("Update Params!")
		self._buttons = data.buttons

if __name__ == '__main__':
	rospy.init_node('joystick_button', anonymous=True)
	joy_topic = rospy.get_param("~joy_topic", "/joy")
	controller = Controller(joy_topic)  
	rospy.spin()
