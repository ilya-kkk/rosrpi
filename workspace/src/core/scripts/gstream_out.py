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
        
        # Создаем объект CvBridge для преобразования ROS Image в OpenCV формат
        self.bridge = CvBridge()
        
        # Подписка на топик с изображениями (например, с USB-камеры)
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

        # GStreamer pipeline для отправки видео по UDP (отправка на порт 5000)
        self.gstreamer_command = [
            'gst-launch-1.0',
            'appsrc', 'format=time', 'is-live=true', 'block=true', 'do-timestamp=true',
            'caps=video/x-raw,format=BGR,width=640,height=480,framerate=30/1',
            '!', 'videoconvert',
            '!', 'x264enc', 'speed-preset=ultrafast', 'tune=zerolatency', 'bitrate=500',
            '!', 'rtph264pay', 'config-interval=1', 'pt=96',
            '!', 'udpsink', 'host=127.0.0.1', 'port=5000'
        ]

        # Запуск GStreamer в отдельном процессе с передачей данных через stdin
        self.gstreamer_process = subprocess.Popen(self.gstreamer_command, stdin=subprocess.PIPE)

        # Обеспечим корректное завершение процесса при выходе из программы
        atexit.register(self.cleanup)

    def image_callback(self, msg):
        try:
            # Преобразуем ROS Image в формат OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Изменяем размер изображения до 640x480, чтобы соответствовать настройкам pipeline
            cv_image = cv2.resize(cv_image, (640, 480))
            
            # Отправляем данные в stdin процесса GStreamer
            self.gstreamer_process.stdin.write(cv_image.tobytes())
            self.gstreamer_process.stdin.flush()
        except Exception as e:
            rospy.logerr(f"Ошибка при обработке изображения: {e}")

    def cleanup(self):
        """Корректное завершение процесса GStreamer при остановке ноды."""
        if self.gstreamer_process:
            self.gstreamer_process.terminate()
            self.gstreamer_process.wait()

if __name__ == '__main__':
    try:
        publisher = GStreamerImagePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
