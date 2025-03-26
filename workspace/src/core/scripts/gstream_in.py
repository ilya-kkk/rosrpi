#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import time
import os
import subprocess

class GStreamerVideoReceiver:
    def __init__(self, ip='127.0.0.1', port=5005):
        rospy.init_node('gstreamer_video_receiver', anonymous=True)
        rospy.logwarn("IN инитится 1/4")

        self.bridge = CvBridge()
        rospy.logwarn("IN инитится 2/4")

        self.image_pub = rospy.Publisher('/nn_image', Image, queue_size=10)
        rospy.logwarn("IN инитится 3/4")

        self.input_pipeline = 'udpsrc port=5001 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! h264parse ! decodebin ! videoconvert ! appsink'
        self.cap = cv2.VideoCapture(self.input_pipeline, cv2.CAP_GSTREAMER)

        rospy.logwarn("IN УСПЕШНО 4/4 заинитился")


    def capture_and_publish(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                rospy.logwarn("IN пришло")
            else:
                rospy.logerr("IN не пришло")

            try:
                # Преобразуем кадр в формат ROS Image
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    
                # Публикуем изображение в топик /nn_image
                self.image_pub.publish(ros_image)
                rospy.logwarn("IN ЗАЕБИСЬ ОНО ПАШЕТ!!!")

            except Exception as e:
                    rospy.logwarn(f"IN Ошибка при преобразовании изображения: {e}")

if __name__ == '__main__':
    try:
        video_receiver = GStreamerVideoReceiver('127.0.0.1', 5005)
        video_receiver.capture_and_publish()
    except rospy.ROSInterruptException:
        pass
