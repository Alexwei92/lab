#!/usr/bin/env python
# Hovering to certain height from current x and y
# Author: Peng Wei
# Last Update: 10/03/2018

import rospy
import tf
from geometry_msgs.msg import PoseStamped

class Start_Point():
	def __init__(self,pose_topic):
		msg = rospy.wait_for_message(pose_topic, PoseStamped)
		self.x = msg.pose.position.x
		self.y = msg.pose.position.y
		self.z = msg.pose.position.z

		#self.sub_once = rospy.Subscriber(pose_topic, PoseStamped, self.read_init)

	#def read_init(self, data):
		#self.x = data.pose.position.x
		#self.y = data.pose.position.y
		#self.z = data.pose.position.z


if __name__ == '__main__':
	rospy.init_node('pose', anonymous=True)
	name = rospy.get_param("~name","goal")     # publish to
	trim = rospy.get_param("~trim","/vrpn_client_node/cf1/pose")
	height = rospy.get_param("~height", 0.5)
	r = rospy.get_param("~rate", 30)	# frequency
	rate = rospy.Rate(r)

	start_point = Start_Point(trim)
	rospy.sleep(1.)
	msg = PoseStamped()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = "/static"
	msg.pose.position.x = start_point.x
	msg.pose.position.y = start_point.y
	msg.pose.position.z = start_point.z + height
	quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
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