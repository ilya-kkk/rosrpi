#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import logging
from core.gstreamer_utils.gstreamer_utils import GstSubscriber

# Настройка логирования
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class GStreamerVideoReceiver:
    def __init__(self, ip='127.0.0.1', port=5001):
        rospy.init_node('gstreamer_video_receiver', anonymous=True)
        logger.info("IN инитится 1/4")

        self.bridge = CvBridge()
        logger.info("IN инитится 2/4")

        self.image_pub = rospy.Publisher('/nn_image', Image, queue_size=10)
        logger.info("IN инитится 3/4")

        # Создаем GStreamer подписчик
        self.subscriber = GstSubscriber(ip=ip, port=port, width=640, height=480)
        logger.info("IN УСПЕШНО 4/4 заинитился")

    def capture_and_publish(self):
        while not rospy.is_shutdown():
            try:
                frame = self.subscriber.get_frame(timeout=0.1)
                if frame is None:
                    continue

                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(ros_image)
                logger.info("IN Кадр получен и опубликован")

            except Exception as e:
                logger.error(f"IN Ошибка при обработке изображения: {e}")
                time.sleep(0.1)  # Пауза перед следующей попыткой

    def cleanup(self):
        if hasattr(self, 'subscriber'):
            self.subscriber.stop()

if __name__ == '__main__':
    try:
        video_receiver = GStreamerVideoReceiver()
        video_receiver.subscriber.start()
        video_receiver.capture_and_publish()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'video_receiver' in locals():
            video_receiver.cleanup()
