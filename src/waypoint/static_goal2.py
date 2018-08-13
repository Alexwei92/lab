#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

class Start_Point():
	def __init__(self, pose_topic):
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		rospy.Subscriber(pose_topic, PoseStamped, self.read_init)

	def read_init(self, data):
		self.x = data.pose.position.x
		self.y = data.pose.position.y
		self.z = data.pose.position.z



if __name__ == '__main__':
	rospy.init_node('static_pose', anonymous=True)
	name = rospy.get_param("~name","/goal")     # publish to
	trim = rospy.get_param("~pose")
	r = rospy.get_param("~rate", 100)	# frequency
	rate = rospy.Rate(r)

	start_point = Start_Point(trim)

	#x = rospy.get_param("~x", -0.1)	# meter
	#y = rospy.get_param("~y", 0.0)	# meter
	#z = rospy.get_param("~z", 1.0)	# meter
	#yaw = rospy.get_param("~yaw", 0.0)  # radian
 
	msg = PoseStamped()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	#start_time = msg.header.stamp
	msg.header.frame_id = "/static"
	msg.pose.position.x = start_point.x
	msg.pose.position.y = start_point.y
	msg.pose.position.z = start_point.z + 0.4
	quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
	msg.pose.orientation.x = quaternion[0]
	msg.pose.orientation.y = quaternion[1]
	msg.pose.orientation.z = quaternion[2]
	msg.pose.orientation.w = quaternion[3]
	
	pub = rospy.Publisher(name, PoseStamped, queue_size=5)

	
	while not rospy.is_shutdown():
		msg.header.seq += 1
		msg.header.stamp = rospy.Time.now()
		#rospy.loginfo("seq:%i, dt:%f", msg.header.seq, msg.header.stamp.to_sec() - start_time.to_sec())
		pub.publish(msg)
		rate.sleep()
