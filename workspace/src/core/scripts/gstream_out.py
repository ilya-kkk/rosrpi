#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess
import atexit

class GStreamerImagePublisher:
    def __init__(self):
        # Инициализация ROS-ноды
        rospy.init_node('gstreamer_image_publisher', anonymous=True)
        
        # Создаем объект CvBridge для преобразования ROS Image в OpenCV
        self.bridge = CvBridge()
        
        # Подписываемся на топик /camera1/image_raw
        self.image_sub = rospy.Subscriber('/camera1/image_raw', Image, self.image_callback)

        # GStreamer pipeline для передачи видео через UDP
        self.gstreamer_command = [
            'gst-launch-1.0',
            'appsrc', 'format=time', 'is-live=true', 'block=true', 'do-timestamp=true',
            'caps=video/x-raw,format=BGR,width=640,height=480,framerate=30/1',
            '!', 'videoconvert',
            '!', 'x264enc', 'tune=zerolatency', 'bitrate=500',
            '!', 'rtph264pay', 'pt=96',
            '!', 'udpsink', 'host=192.168.0.108', 'port=5000'
        ]

        # Запускаем GStreamer в subprocess
        self.gstreamer_process = subprocess.Popen(self.gstreamer_command, stdin=subprocess.PIPE)

        # Закрываем процесс при завершении программы
        atexit.register(self.cleanup)

    def image_callback(self, msg):
        try:
            # Преобразуем сообщение ROS Image в OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Изменяем размер изображения до 640x480 (чтобы соответствовать GStreamer pipeline)
            cv_image = cv2.resize(cv_image, (640, 480))

            # Отправляем изображение в GStreamer через stdin
            self.gstreamer_process.stdin.write(cv_image.tobytes())
            self.gstreamer_process.stdin.flush()
        
        except Exception as e:
            rospy.logerr(f"Ошибка при обработке изображения: {e}")

    def cleanup(self):
        """Закрытие процесса GStreamer при завершении ноды."""
        if self.gstreamer_process:
            self.gstreamer_process.terminate()
            self.gstreamer_process.wait()

if __name__ == '__main__':
    try:
        # Создаем объект ноды и запускаем ее
        gstreamer_publisher = GStreamerImagePublisher()
        
        # Ожидаем, пока нода не будет остановлена
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass
