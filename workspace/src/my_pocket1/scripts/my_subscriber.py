#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def pose_callback(msg):
    rospy.loginfo(msg)

if __name__ == '__main__':
    rospy.init_node("pose_subscriber")
    rospy.loginfo("Node started...")
    rate = rospy.Rate(10)
    subscriber = rospy.Subscriber("/turtle1/pose", Pose, callback = pose_callback)
    rospy.spin()


