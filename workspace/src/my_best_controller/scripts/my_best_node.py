#!/usr/bin/env python3

import time
import math as m
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Get frequency from parameter server, default to 30
frequency = int(rospy.get_param('frequency', '30'))

## Parent error class
class Error():
    def calc(self, x, y, tx, ty):
        return 0

## Child error class for angular error
class AngErr(Error):
    def calc(self, x, y, tx, ty):
        return m.atan2(tx - x, ty - y)

## Child error class for linear error
class LinErr(Error): 
    def calc(self, x, y, tx, ty):
        return m.sqrt((tx - x)**2 + (ty - y)**2)

class PID:
    def __init__(self):
        self.__Kp = 0
        self.__Ki = 0
        self.__Kd = 0
        self.__delay = 0.0

    def set_coeff(self, p, i, d):
        self.__Kp = p
        self.__Ki = i
        self.__Kd = d

    def set_frequency(self, freq):
        self.__delay = 1 / freq

    def calc(self, input_error):
        output_signal = 0
        output_signal += input_error * self.__Kp
        output_signal += input_error * self.__Ki / self.__delay
        output_signal += input_error * self.__Kd * self.__delay
        return output_signal

class SimpleController:
    def __init__(self):
        rospy.logerr("\033[31mSIMPLE CONTROLLER INIT...")
        global frequency

        # PID controllers for linear and angular velocities
        self.linear_vel_reg = PID()
        self.linear_vel_reg.set_coeff(0.1, 0, 0)
        self.linear_vel_reg.set_frequency(frequency)

        self.angular_vel_reg = PID()
        self.angular_vel_reg.set_coeff(0.1, 0, 0)
        self.angular_vel_reg.set_frequency(frequency)

        rospy.init_node('simple_controller', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        #self.cmd_vel_pub = 0 # controller -> sim output
        #subscriber = 0
        #target_sub = 0 # feedback

        self.rate = rospy.Rate(frequency)

        self.target_x = 0
        self.target_y = 0
        self.x = 0
        self.y = 0
        self.ang_err = 0
        self.lin_err = 0
        rospy.logerr("\033[31mSIMPLE CONTROLLER INIT DONE")

    def target_callback(self, msg: Pose):
        self.target_x, self.target_y = msg.x, msg.y

    def pose_callback(self, msg: Pose):
        self.update_control(msg.x, msg.y)

    def update_control(self, x, y):
        self.x, self.y = x, y
        self.ang_err = AngErr().calc(x, y, self.target_x, self.target_y)
        self.lin_err = LinErr().calc(x, y, self.target_x, self.target_y)

    def spin(self):
        while not rospy.is_shutdown():

            self.rate.sleep()

    def go_to(self):
        while not (self.target_x == self.x and self.target_y == self.y):
            rospy.logerr("\033[31m go_to start...")
            output_msg = Twist()
            output_msg.linear.x = self.linear_vel_reg.calc(self.lin_err)
            output_msg.angular.z = self.angular_vel_reg.calc(self.ang_err)
            self.cmd_vel_pub.publish(output_msg)
            self.rate.sleep()
            rospy.logerr("\033[31m go_to end")

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


class MasterController(SimpleController):
    def __init__(self):
        super().__init__(self)
        rospy.logerr("\033[31mMASTER TURTLE INIT...")
        self.cmd_vel_pub = rospy.Publisher('ns1_470122/turtle1/cmd_vel', Twist, queue_size=1)  # controller -> sim output
        subscriber = rospy.Subscriber("ns1_470122/turtle1/pose", Pose, self.pose_callback) 
        self.target_publisher = rospy.Publisher('ns1_470122/targets_topic', Pose, queue_size=1)
        target_sub = rospy.Subscriber("ns1_470122/targets_topic", Pose, self.target_callback)

        rospy.logerr("\033[31mMASTER TURTLE INIT DONE")

    def spin(self):
        target_list = [(2, 3), (4, 2), (5, 4)]
        for target in target_list:
            target_out_msg = Pose()
            target_out_msg.x = target[0]
            target_out_msg.y = target[1]
            self.target_publisher.publish(target_out_msg)
            self.go_to()


class SlaveController(SimpleController):
    def __init__(self):
        super().__init__(self)
        rospy.logerr("\033[31mSLAVE TURTLE INIT...")
        self.cmd_vel_pub = rospy.Publisher('/ns1_470122/sim/cmd_vel', Twist, queue_size=1)  # controller -> sim output
        subscriber = rospy.Subscriber("~turtlesim/pose", Pose, self.pose_callback) 
        target_sub = rospy.Subscriber("/ns2_470122/sim/pose", Pose, self.target_callback)

        rospy.logerr("\033[31mSLAVE TURTLE INIT DONE")

    def spin(self):
        while not rospy.is_shutdown():
            msg = Twist()
            msg.linear.x = 1
            msg.angular.z = 1
            cmd_vel_pub.publish(msg)
            self.rate.sleep()

rospy.logerr("\033[31mSTART NODE...")
rospy.logerr(rospy.get_namespace() )

if __name__ == '__main__':
    if rospy.get_namespace() == '/ns1_470122/':
        simple_mover = MasterController()
        
    elif rospy.get_namespace() == "/ns2_470122/":
        simple_mover = SlaveController()

    else: 
        rospy.logerr("\033[31 get_namespace НЕ сработал")
  
        

rospy.logerr("\033[31mSTART SPIN...")
simple_mover.spin()
rospy.logerr("\033[31mEND PROGRAM")
