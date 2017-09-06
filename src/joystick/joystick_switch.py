#!/usr/bin/env python
# switch functions
# Author: Peng Wei

import rospy

from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

def switch():
	rospy.loginfo("enable!")
	return True

class Switch():
	def __init__(self):
		self.index = 0
		self.msg = PoseStamped()
		self.msg.header.seq = 0
		self.msg.header.frame_id = "/world"
		self.msg.header.stamp = rospy.Time.now()

	def _switch(self,data):
		if data.buttons[3] == 1: 
			self.index += 1
     		#rospy.loginfo("INDEX = %i", self.index)

# msg_tmp1 = PoseStamped()
# msg_tmp2 = PoseStamped()

# def copy(PoseStamped source):
# 	if source.header.frame_id == "/static":
# 		msg_tmp1 = source
	
# 	else if source.header.frame_id == "/hover":
# 		msg_tmp2 = source
# 	else:
# 		rospy.loginfo("Error fram_id!")



if __name__ == '__main__':
	rospy.init_node('switch')
	#node1 = rospy.get_param('~node1','goal1')
	#node2 = rospy.get_param('~node2','goal2')
	target = rospy.get_param('~name','goal')

	# rospy.Subscriber(node1, PoseStamped, copy)	
	# rospy.Subscriber(node2, PoseStamped, copy)
	
	goal = Switch()
	rospy.Subscriber('/joy', Joy, goal._switch)
	# msg = msg_tmp1
	
	pub = rospy.Publisher(target, PoseStamped, queue_size=5)
	r = rospy.get_param("~rate", 50)
	rate = rospy.Rate(r)

	while not rospy.is_shutdown():
		pub.publish(goal.msg)
		rospy.get_service('switch')
		rate.sleep()

