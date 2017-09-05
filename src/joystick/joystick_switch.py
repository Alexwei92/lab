#!/usr/bin/env python
# switch functions
# Author: Peng Wei

import rospy

from std_srvs import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

msg = PoseStamped()
msg_tmp1 = PoseStamped()
msg_tmp2 = PoseStamped()

def copy(PoseStamped source):

	if source.header.frame_id == "/world1":
		msg_tmp1 = source
	
	else if source.header.frame_id == "/world2":
		msg_tmp2 = source
	else:
		rospy.loginfo("Error fram_id!")


def switch(Empty.Request& req, Empty.Response& res)
    {
        rospy.loginfo("Switch to xxx!");
        return true;
    } 


if __name__ == '__main__':
	rospy.init_node('switch')
	node1 = rospy.get_param('~node1','pose1')
	node2 = rospy.get_param('~node2','pose2')
	target = rospy.get_param('~name','goal')

	rospy.Subscriber(node1, PoseStamped, copy)	
	rospy.Subscriber(node2, PoseStamped, copy)
	msg = msg_tmp1
	msg.header.frame_id = "/world"
	pub = rospy.Publisher(target, PoseStamped, queue_size=5)
	r = rospy.get_param("~rate", 50)
	rate = rospy.Rate(r)

	while not rospy.is_shutdown():
		pub.publish(msg)
		rate.sleep()

