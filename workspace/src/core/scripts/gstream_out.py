#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class GStreamerImagePublisher:
    def __init__(self):
        rospy.init_node('gstreamer_image_publisher', anonymous=True)
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

        self.output_pipeline = (
            "appsrc ! videoconvert ! "
            "x264enc speed-preset=ultrafast tune=zerolatency bitrate=500 ! "
            "rtph264pay config-interval=1 pt=96 ! "
            "udpsink host=127.0.0.1 port=5000"
        )

        self.out = cv2.VideoWriter(self.output_pipeline, cv2.CAP_GSTREAMER, 0, 30, (640, 480), True)

        if not self.out.isOpened():
            rospy.logerr("Ошибка открытия GStreamer VideoWriter")
            self.out = None  # Обнуляем переменную, чтобы не использовать нерабочий объект
        else:
            rospy.logwarn("УСПЕШНО открыт GStreamer VideoWriter")

    def image_callback(self, msg):
        try:
            if self.out is None:  # Проверяем, что self.out инициализирован
                rospy.logwarn("GStreamer VideoWriter не был инициализирован, пропускаем кадр.")
                return
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.resize(cv_image, (640, 480))
            
            self.out.write(cv_image)

        except Exception as e:
            rospy.logerr(f"Ошибка при обработке изображения: {e}")

if __name__ == '__main__':
    try:
        publisher = GStreamerImagePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
