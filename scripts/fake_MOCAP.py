#!/usr/bin/env python

# Fake MOCAP output

import rospy
from geometry_msgs.msg import PoseStamped
import tf, math


if __name__ == '__main__':
    rospy.init_node('fake_mocap')
    rate = rospy.Rate(240)

    pub = rospy.Publisher('/vrpn_client_node/cf1/pose', PoseStamped, queue_size=1)

    fake = PoseStamped()
    fake.pose.position.x = 0.25
    fake.pose.position.y = 0.11
    fake.pose.position.z = 0.04

    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 5*math.pi/180.0)
    fake.pose.orientation.x = quaternion[0]
    fake.pose.orientation.y = quaternion[1]
    fake.pose.orientation.z = quaternion[2]
    fake.pose.orientation.w = quaternion[3]

    while not rospy.is_shutdown():
        fake.header.seq += 1
        fake.header.stamp = rospy.Time.now()
        pub.publish(fake)
        rate.sleep()