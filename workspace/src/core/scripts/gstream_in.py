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
    def __init__(self, ip='127.0.0.1', port=5001):
        rospy.init_node('gstreamer_video_receiver', anonymous=True)
        rospy.logwarn("IN инитится 1/4")

        self.bridge = CvBridge()
        rospy.logwarn("IN инитится 2/4")

        self.image_pub = rospy.Publisher('/nn_image', Image, queue_size=10)
        rospy.logwarn("IN инитится 3/4")

        # Изменяем pipeline для получения JPEG кадров от YOLO
        self.input_pipeline = 'udpsrc port=5001 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)JPEG, payload=(int)26" ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink'
        self.cap = cv2.VideoCapture(self.input_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            rospy.logerr("IN Ошибка: Не удалось открыть входной поток GStreamer")
            return

        rospy.logwarn("IN УСПЕШНО 4/4 заинитился")

    def capture_and_publish(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                # rospy.logerr("IN Ошибка чтения кадра")
                continue

            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(ros_image)
                rospy.logwarn("IN Кадр получен и опубликован")

            except Exception as e:
                rospy.logerr(f"IN Ошибка при обработке изображения: {e}")

if __name__ == '__main__':
    try:
        video_receiver = GStreamerVideoReceiver()
        video_receiver.capture_and_publish()
    except rospy.ROSInterruptException:
        pass
