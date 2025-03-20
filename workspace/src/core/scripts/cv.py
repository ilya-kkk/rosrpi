#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Bool, Float64
import cv2
import numpy as np
from cv_bridge import CvBridge

class WhiteRegionDetector:
    def __init__(self):
        # Подписка на исходное изображение
        sub_topic_name = "/camera1/image_raw"
        self.camera_subscriber = rospy.Subscriber(sub_topic_name, Image, self.camera_cb)

        # Паблишеры для обработанных изображений и данных
        self.image_publisher = rospy.Publisher("/white_region", Image, queue_size=10)
        self.value_publisher = rospy.Publisher("/value", Float64, queue_size=10)
        self.flag_publisher = rospy.Publisher("/flag", Bool, queue_size=10)

        # Конвертер между ROS и OpenCV
        self.bridge = CvBridge()

    def camera_cb(self, data):
        try:
            # Преобразуем изображение из формата ROS в OpenCV
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

            # Преобразуем в градации серого
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Применяем бинаризацию
            _, binary_image = cv2.threshold(gray_frame, 50, 100, cv2.THRESH_BINARY)

            height, width = binary_image.shape
            upper = 0.5
            lower = 0.25
            binary_image[0:int(height * upper), :] = 0
            binary_image[int((1-lower) * height):height, :] = 0

            # Находим контуры
            contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Флаг для наличия белой области
            white_region_found = False
            center_of_white_region = None
            for contour in contours:
                if cv2.contourArea(contour) > 100:  # Игнорируем слишком маленькие области
                    # Вычисляем моменты контура
                    M = cv2.moments(contour)
                    if M["m00"] > 0:  # Проверяем, что площадь ненулевая
                        # Вычисляем координаты центра массы
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])

                        # Наложение линии от центра белой области до центра изображения
                        height, width = frame.shape[:2]
                        center_image = (width // 2, height)

                        # Рисуем центр белой области и линию
                        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)  # Красная точка
                        cv2.line(frame, center_image, (cX, cY), (0, 255, 0), 2) 

                        cv2.line(frame, (0, int(upper * height)), (width, int(upper * height)), (255, 0, 0), 2)
                        cv2.line(frame, (0, int((1 - lower) * height)), (width, int((1 - lower) * height)), (255, 0, 0), 2)

                        distance_x = cX - center_image[0]
                        self.value_publisher.publish(distance_x)
                        white_region_found = True

            # Публикуем флаг, если белая область найдена
            self.flag_publisher.publish(white_region_found)
            # rospy.loginfo(f"Dist = {distance_x}")
            # rospy.loginfo(f"flag = {white_region_found}") 
            
            # Публикуем изображение с наложением центра и линии
            self.publish_image(self.image_publisher, frame, encoding="bgr8")
            
        except Exception as e:
            rospy.logerr(f"Error in camera_cb: {e}")

    def publish_image(self, publisher, image, encoding):
        """
        Публикуем изображение в заданный топик.
        """
        try:
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
            ros_image.header = Header()
            ros_image.header.stamp = rospy.Time.now()
            publisher.publish(ros_image)
        except Exception as e:
            rospy.logerr(f"Error publishing image: {e}")

if __name__ == '__main__':
    rospy.init_node("white_region_detector")
    WhiteRegionDetector()
    rospy.spin()
