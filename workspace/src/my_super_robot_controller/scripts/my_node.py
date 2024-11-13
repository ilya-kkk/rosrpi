#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from my_super_robot_controller.msg import my_Num

def pose_callback(pose):
    mymsg = my_Num()
    mymsg.x = pose.x
    mymsg.y = pose.y
    publisher.publish(mymsg)
    rospy.loginfo('('+ str(mymsg.x) + ", " + str(mymsg.y) + ")")

if __name__ == '__main__':
    rospy.init_node("turtle_super_node")
    rospy.loginfo("Node started...")
    # rate = rospy.Rate(2)
    subscriber = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    publisher = rospy.Publisher("/position2d", Twist, queue_size=10)
    rospy.spin()
