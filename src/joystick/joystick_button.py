#!/usr/bin/env python
# Joystick buttons mapping
# Author: Peng Wei

import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

class Controller():
    def __init__(self, joy_topic):
	rospy.wait_for_service('emergency')
        self._emergency = rospy.ServiceProxy('emergency', Empty)
	rospy.loginfo("created emergency service!")
	
	rospy.wait_for_service('land')
        self._land = rospy.ServiceProxy('land', Empty)
 	rospy.loginfo("created land service!")

	rospy.wait_for_service('takeoff')
        self._takeoff = rospy.ServiceProxy('takeoff', Empty)
	rospy.loginfo("created takeoff service!")
	
	rospy.wait_for_service('switch')
	self._switch = rospy.ServiceProxy('switch', Empty)
	rospy.loginfo("created switch service!")

        # subscribe to the joystick at the end to make sure that all required
        # services were found
        self._buttons = None
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)

    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                if i == 0 and data.buttons[i] == 1:
                    self._land()
		    rospy.loginfo("Landing requested!")
                if i == 1 and data.buttons[i] == 1:
                    self._emergency()
		    rospy.loginfo("Emergency requested!")
                if i == 2 and data.buttons[i] == 1:
                    self._takeoff()
		    rospy.loginfo("TakeOff requested!")
 		if i == 3 and data.buttons[i] == 1:
		    self._switch()
		    rospy.loginfo("Switch Requested!")

        self._buttons = data.buttons

if __name__ == '__main__':
    rospy.init_node('joystick_button', anonymous=True)
    joy_topic = rospy.get_param("~joy_topic", "joy")
    controller = Controller(joy_topic)
    rospy.spin()
