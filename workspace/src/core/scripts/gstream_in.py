#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess

class GStreamerVideoReceiver:
    def __init__(self):
        # Инициализация ROS ноды
        rospy.init_node('gstreamer_video_receiver', anonymous=True)
        
        # Создаем объект для преобразования изображения между ROS и OpenCV
        self.bridge = CvBridge()
        
        # Публикуем изображения в топик /nn_image
        self.image_pub = rospy.Publisher('/nn_image', Image, queue_size=10)
        
        # Запуск GStreamer для приема видеопотока
        self.gstreamer_command = (
            'gst-launch-1.0', 
            'udpsrc', 
            'address=127.0.0.1',  # IP адрес источника потока
            'port=5000', 
            'caps="application/x-rtp, media=video, payload=96"', 
            '!', 
            'rtph264depay', 
            '!', 
            'avdec_h264', 
            '!', 
            'videoconvert', 
            '!', 
            'appsink', 
            'sync=false'
        )

        # Запуск процесса GStreamer для захвата видео
        self.gstreamer_process = subprocess.Popen(self.gstreamer_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.cap = cv2.VideoCapture(self.gstreamer_process.stdout)
        
    def capture_and_publish(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                try:
                    # Преобразуем кадр в формат ROS Image
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    
                    # Публикуем изображение в топик /nn_image
                    self.image_pub.publish(ros_image)
                    
                except Exception as e:
                    rospy.logerr(f"Ошибка при преобразовании изображения: {e}")
            else:
                rospy.logwarn("Не удалось захватить кадр!")

if __name__ == '__main__':
    try:
        video_receiver = GStreamerVideoReceiver()
        video_receiver.capture_and_publish()
    except rospy.ROSInterruptException:
        pass
