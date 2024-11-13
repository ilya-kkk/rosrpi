#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import Int32 

if __name__ == '__main__':
    rospy.init_node("node1")
    rospy.loginfo("Node 1 started...")
    rate = rospy.Rate(10)
    pub = rospy.Publisher("/connection/num_1", Int32, queue_size=10)
    msg = 0

    while not rospy.is_shutdown():
        msg = random.randint(0, 10)
        output_msg = Int32(msg)
        pub.publish(output_msg)
        rospy.loginfo(f'num3 =  {output_msg.data}')
        rate.sleep()
