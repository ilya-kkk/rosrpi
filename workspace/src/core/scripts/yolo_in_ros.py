#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO

model_path = "/workspace/src/core/scripts/yolo8n.pt"
if not os.path.exists(model_path):
    rospy.loginfo("Model not found! Downloading...")
    # Загружаем модель с указанием устройства GPU
    model = YOLO('yolo8n.pt', device="cuda")
else:
    model = YOLO(model_path, device="cuda")

# Целевые классы для детекции
target_classes = [0, 2, 14, 15]

def filter_detections(results, target_classes):
    """Фильтрация обнаруженных объектов по целевым классам."""
    filtered = []
    boxes = results[0].boxes
    for box in boxes:
        if int(box.cls.item()) in target_classes:
            filtered.append(box)
    return filtered

class YoloNode:
    def __init__(self):
        rospy.init_node('yolo_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/image_nn', Image, queue_size=1)
        rospy.loginfo("YOLO node initialized and ready on GPU.")

    def image_callback(self, msg):
        try:
            # Преобразуем ROS Image в OpenCV изображение
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.loginfo("Received image for processing.")

            # Запускаем инференс на GPU
            results = model(frame)
            filtered_boxes = filter_detections(results, target_classes)

            # Рисуем bounding box'ы и метки на изображении
            for box in filtered_boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = box.conf.item()
                cls = int(box.cls.item())
                label = f"{model.names[cls]} {conf:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Преобразуем обработанное изображение обратно в ROS сообщение и публикуем
            processed_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(processed_msg)
            rospy.loginfo("Processed image published to /image_nn.")
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == "__main__":
    try:
        node = YoloNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("YOLO node terminated.")
