#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node("test_node_2")
    rospy.loginfo("Node started...")
    rate = rospy.Rate(10)
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    msg = Twist()

    while not rospy.is_shutdown():
        msg.linear.x = 1
        msg.angular.z = 0.5
        pub.publish(msg)
        rate.sleep()

    # rospy.logwarn("Hello world worn")
    # rospy.logerr("Hello world err")

