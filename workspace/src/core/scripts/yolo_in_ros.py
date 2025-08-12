#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO


model_path = "/workspace/src/core/scripts/yolo8n.pt"
if not os.path.exists(model_path):
    print("Model not found! Downloading...")
    model = YOLO('yolo8n.pt')
else:
    model = YOLO(model_path)

# Target classes for detection
target_classes = [0, 2, 14, 15]

def filter_detections(results, target_classes):
    """Filter detected objects by target classes."""
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
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/image_nn', Image, queue_size=1)
        rospy.loginfo("YOLO node initialized and ready.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.loginfo("Received image for processing.")

            # Apply YOLO model for object detection
            results = model(frame)
            filtered_boxes = filter_detections(results, target_classes)

            # Draw bounding boxes and labels on the frame
            for box in filtered_boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = box.conf.item()
                cls = int(box.cls.item())
                label = f"{model.names[cls]} {conf:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Convert processed frame back to ROS Image message and publish
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
