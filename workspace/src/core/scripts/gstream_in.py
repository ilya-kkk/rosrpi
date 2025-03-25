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
        rospy.logwarn("Из нейронки в рос инитится")
        # Инициализация ROS ноды
        rospy.init_node('gstreamer_video_receiver', anonymous=True)
        
        # Создаем объект для преобразования изображения между ROS и OpenCV
        self.bridge = CvBridge()
        
        # Публикуем изображения в топик /nn_image
        self.image_pub = rospy.Publisher('/nn_image', Image, queue_size=10)
        
        self.ip = ip
        self.port = port

        # Ожидание доступности порта перед запуском GStreamer
        # while not self.is_port_open():
        #     rospy.logwarn(f"Не удалось подключиться к {self.ip}:{self.port}. Повторная попытка через 1 секунду...")
        #     time.sleep(1)
        
        # Проверка и запуск GStreamer
        # self.ensure_gstreamer_running()

        # GStreamer pipeline для получения видео
        # gst_pipeline = (
        #     "udpsrc port=5005 ! "
        #     "application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 ! "
        #     "rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false"
        #     )
        input_pipeline = 'udpsrc port=5000 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! appsink'


        # Используем cv2.VideoCapture с GStreamer
        while True:
            self.cap = cv2.VideoCapture(input_pipeline, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                break  # Выходим из цикла, если видеопоток успешно открыт
            rospy.logwarn("Ошибка: Не удалось открыть видеопоток. Повторная попытка...")

        # if not self.cap.isOpened():
        #     rospy.logerr("Не удалось открыть видеопоток через GStreamer.")
        #     self.cap = None
        rospy.logwarn("Из нейронки в рос заинитилось")

    def is_port_open(self):
        """Проверка доступности порта с использованием socket."""
        try:
            with socket.create_connection((self.ip, self.port), timeout=1):
                return True
        except (socket.timeout, socket.error):
            return False

    def is_gstreamer_running(self):
        """Проверка, запущен ли процесс GStreamer."""
        try:
            # Проверяем процессы с помощью командной строки
            output = subprocess.check_output(["pgrep", "-f", "gstreamer"], stderr=subprocess.PIPE)
            return len(output.strip()) > 0
        except subprocess.CalledProcessError:
            return False

    def ensure_gstreamer_running(self):
        """Обеспечиваем, что процесс GStreamer запущен. Повторная попытка, если нет."""
        while not self.is_gstreamer_running():
            rospy.logwarn("GStreamer не запущен. Ожидание...")
            time.sleep(1)
        
        rospy.loginfo("GStreamer процесс успешно запущен.")

    def capture_and_publish(self):
        # if self.cap is None:
        #     rospy.logerr("Камера не была успешно инициализирована.")
        #     return

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
    
            try:
                # Преобразуем кадр в формат ROS Image
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    
                # Публикуем изображение в топик /nn_image
                self.image_pub.publish(ros_image)
                rospy.logwarn("ЗАЕБИСЬ ОНО ПАШЕТ!!!")

            except Exception as e:
                    rospy.logwarn(f"Ошибка при преобразовании изображения: {e}")

if __name__ == '__main__':
    try:
        video_receiver = GStreamerVideoReceiver('127.0.0.1', 5001)
        video_receiver.capture_and_publish()
    except rospy.ROSInterruptException:
        pass
