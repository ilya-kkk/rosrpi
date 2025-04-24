#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import logging
from core.gstreamer_utils.gstreamer_utils import GstPublisher

# Настройка логирования
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class GStreamerImagePublisher:
    def __init__(self):
        rospy.init_node('gstreamer_image_publisher', anonymous=True)
        logger.info("OUT инитится 1/4")
        
        self.bridge = CvBridge()
        logger.info("OUT инитится 2/4")

        # Создаем GStreamer публикатор
        self.publisher = GstPublisher(ip="127.0.0.1", port=5000, width=640, height=480)
        logger.info("OUT инитится 3/4")

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        logger.info("OUT УСПЕШНО 4/4 заинитился")

    def image_callback(self, msg):
        try:
            if not self.publisher.is_running:
                logger.warning("OUT GStreamer Publisher не запущен, пропускаем кадр.")
                return
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.resize(cv_image, (640, 480))
            
            if self.publisher.publish_frame(cv_image):
                logger.info("OUT УСПЕШНО отправлен кадр")
            else:
                logger.warning("OUT Не удалось отправить кадр")

        except Exception as e:
            logger.error(f"OUT Ошибка при обработке изображения: {e}")

    def cleanup(self):
        if hasattr(self, 'publisher'):
            self.publisher.stop()

if __name__ == '__main__':
    try:
        publisher = GStreamerImagePublisher()
        publisher.publisher.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'publisher' in locals():
            publisher.cleanup()
