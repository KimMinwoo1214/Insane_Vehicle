#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32MultiArray
from cv_bridge import CvBridge

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        self.bbox_pub = self.create_publisher(Float32MultiArray, "/bbox_centers", 10)
        self.tunnel_pub = self.create_publisher(Int32, "/tunnel_mode", 10)
        self.create_subscription(Image, "/video_frames", self.image_callback, 10)
        
        try:
            self.model = YOLO('../weights/YOLO_0216.pt')
        except Exception as e:
            self.get_logger().error(f'모델 로드 실패: {str(e)}')
            raise

        self.bridge = CvBridge()
        self.TUNNEL_SIZE_THRESHOLD = 100

    def image_callback(self, msg):
        try:
            current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model.predict(
                source=current_frame,
                conf=0.25,
                iou=0.45,
                verbose=False
            )
            
            centers = []
            tunnel_mode = 0

            if results and len(results) > 0:
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        xyxy = box.xyxy.cpu().numpy().flatten()
                        x1, y1, x2, y2 = map(int, xyxy)

                        label = int(box.cls.item()) if hasattr(box.cls, 'item') else int(box.cls)

                        if label in [0, 1]:
                            center_x = (x1 + x2) / 2
                            center_y = (y1 + y2) / 2
                            centers.extend([center_x, center_y])
                        elif label == 2:
                            width = x2 - x1
                            height = y2 - y1
                            if width > self.TUNNEL_SIZE_THRESHOLD and height > self.TUNNEL_SIZE_THRESHOLD:
                                tunnel_mode = 1
            
            if centers:
                bbox_msg = Float32MultiArray()
                bbox_msg.data = centers
                self.bbox_pub.publish(bbox_msg)

            self.tunnel_pub.publish(Int32(data=tunnel_mode))

            cv2.imshow("Camera View", current_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main():
    rclpy.init()
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
