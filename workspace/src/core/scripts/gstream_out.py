#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess

class GStreamerImagePublisher:
    def __init__(self):
        # Инициализация ROS ноды
        rospy.init_node('gstreamer_image_publisher', anonymous=True)
        
        # Создаем объект CvBridge для преобразования ROS Image в OpenCV
        self.bridge = CvBridge()
        
        # Подписываемся на топик /camera1/image_raw
        self.image_sub = rospy.Subscriber('/camera1/image_raw', Image, self.image_callback)
        
        # Параметры GStreamer (трансляция на порт 5000)
        self.gstreamer_command = [
            'gst-launch-1.0', 
            'appsrc', 
            '!', 
            'videoconvert', 
            '!', 
            'x264enc', 
            'tune=zerolatency', 
            'bitrate=500', 
            '!', 
            'rtph264pay', 
            'pt=96', 
            '!', 
            'udpsink', 
            'host=127.0.0.1',  # Используйте нужный IP для получения потока
            'port=5000'
        ]

        # Запускаем GStreamer в subprocess
        self.gstreamer_process = subprocess.Popen(self.gstreamer_command, stdin=subprocess.PIPE)

    def image_callback(self, msg):
        try:
            # Преобразуем сообщение ROS Image в изображение OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Отправляем изображение в GStreamer через stdin
            # Преобразуем изображение в строку байтов и передаем через stdin
            _, buffer = cv2.imencode('.jpg', cv_image)
            self.gstreamer_process.stdin.write(buffer.tobytes())
            self.gstreamer_process.stdin.flush()
        
        except Exception as e:
            rospy.logerr(f"Ошибка при обработке изображения: {e}")

if __name__ == '__main__':
    try:
        # Создаем объект ноды и запускаем ее
        gstreamer_publisher = GStreamerImagePublisher()
        
        # Ожидаем, пока нода не будет остановлена
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass
