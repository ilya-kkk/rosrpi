#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node("pub_cmd_vel_once")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # даём чуть времени на соединение с подписчиками
    rospy.sleep(3.5)

    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    pub.publish(msg)
    rospy.loginfo("Published one /cmd_vel")
    # оставим узел живым немного, чтобы подписчики гарантированно приняли сообщение
    rospy.sleep(0.5)
