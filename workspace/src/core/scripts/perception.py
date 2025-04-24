#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

class DepthMaskProcessor:
    def __init__(self):
        rospy.init_node('depth_mask_processor', anonymous=True)
        
        # Инициализация CvBridge для конвертации между ROS и OpenCV форматами
        self.bridge = CvBridge()
        
        # Подписка на топики
        self.depth_sub = rospy.Subscriber('/d435/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.mask_sub = rospy.Subscriber('/mask', Image, self.mask_callback)
        self.camera_info_sub = rospy.Subscriber('/d435/camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        
        # Публикация облака точек
        self.pointcloud_pub = rospy.Publisher('/filtered_pointcloud', PointCloud2, queue_size=10)
        
        # Хранение последних полученных данных
        self.current_depth = None
        self.current_mask = None
        
        # Параметры камеры
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        rospy.loginfo("Depth Mask Processor node initialized")

    def camera_info_callback(self, msg):
        # Получение параметров камеры из camera_info
        self.fx = msg.K[0]  # focal length x
        self.fy = msg.K[4]  # focal length y
        self.cx = msg.K[2]  # principal point x
        self.cy = msg.K[5]  # principal point y
        rospy.loginfo(f"Camera parameters updated: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def depth_callback(self, msg):
        try:
            # Конвертация ROS Image в numpy array
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.process_data()
        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")

    def mask_callback(self, msg):
        try:
            # Конвертация ROS Image в numpy array
            self.current_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            self.process_data()
        except Exception as e:
            rospy.logerr(f"Error processing mask: {e}")

    def process_data(self):
        if self.current_depth is None or self.current_mask is None or self.fx is None:
            return

        # Приведение размеров к одному формату
        if self.current_depth.shape[:2] != self.current_mask.shape[:2]:
            self.current_mask = cv2.resize(self.current_mask, (self.current_depth.shape[1], self.current_depth.shape[0]))

        # Применение маски к изображению глубины
        masked_depth = cv2.bitwise_and(self.current_depth, self.current_depth, mask=self.current_mask)

        # Создание облака точек
        self.create_pointcloud(masked_depth)

    def create_pointcloud(self, depth_image):
        # Получение размеров изображения
        height, width = depth_image.shape

        # Создание сетки координат
        x, y = np.meshgrid(np.arange(width), np.arange(height))

        # Преобразование координат в реальные размеры
        X = (x - self.cx) * depth_image / self.fx
        Y = (y - self.cy) * depth_image / self.fy
        Z = depth_image

        # Создание массива точек
        points = np.stack([X, Y, Z], axis=-1)
        
        # Фильтрация точек с нулевой глубиной
        valid_points = points[depth_image > 0]
        
        if len(valid_points) == 0:
            return

        # Создание заголовка для PointCloud2
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "d435_camera_link"

        # Создание и публикация облака точек
        pointcloud = point_cloud2.create_cloud_xyz32(header, valid_points)
        self.pointcloud_pub.publish(pointcloud)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        processor = DepthMaskProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass

