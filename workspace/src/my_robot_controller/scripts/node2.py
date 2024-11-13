#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32 

msg1 = 2
msg2 = 1

def pose_callback1(input_msg):
    global msg1
    msg1 = input_msg.data
    #rospy.loginfo("Callback 1")


def pose_callback2(input_msg):
    global msg1
    msg2 = input_msg.data
    #rospy.loginfo("callback 2")


if __name__ == '__main__':
    rospy.init_node("node2")
    rospy.loginfo("Node 2 started...")
    rate = rospy.Rate(10)
    subscriber1 = rospy.Subscriber("/connection/num_1", Int32, callback = pose_callback1)
    subscriber2 = rospy.Subscriber("/connection/num_2", Int32, callback = pose_callback2)
    pub = rospy.Publisher("/result_470122", Int32, queue_size=10)
    output_msg = Int32()


    while not rospy.is_shutdown():
        result = msg1 + msg2 
        output_msg.data = result
        pub.publish(output_msg)
        rospy.loginfo(f'result = {output_msg.data}')
        rate.sleep()
