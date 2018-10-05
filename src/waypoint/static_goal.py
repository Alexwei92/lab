#!/usr/bin/env python
# A static goal published to the drone
# Author: Peng Wei
# Last Update: 10/03/2018

import rospy
import tf
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
	rospy.init_node('static_pose', anonymous=True)
	name = rospy.get_param("~name","/goal")     # publish to
	r = rospy.get_param("~rate", 30)	# frequency
	x = rospy.get_param("~x", 0.0)	# meter
	y = rospy.get_param("~y", 0.0)	# meter
	z = rospy.get_param("~z", 1.0)	# meter
	yaw = rospy.get_param("~yaw", 0.0)  # radian

	rate = rospy.Rate(r)
 
	msg = PoseStamped()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = "/static"
	msg.pose.position.x = x
	msg.pose.position.y = y
	msg.pose.position.z = z
	quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
	msg.pose.orientation.x = quaternion[0]
	msg.pose.orientation.y = quaternion[1]
	msg.pose.orientation.z = quaternion[2]
	msg.pose.orientation.w = quaternion[3]
	
	pub = rospy.Publisher(name, PoseStamped, queue_size=5)

	
	while not rospy.is_shutdown():
		msg.header.seq += 1
		msg.header.stamp = rospy.Time.now()
		pub.publish(msg)
		rate.sleep()
