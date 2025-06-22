#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge

from custom_msgs.msg import BoundingBox2D, BoundingBox2DArray

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.bbox_pub = self.create_publisher(BoundingBox2DArray, "/yolo_bboxes", 10)
        self.tunnel_pub = self.create_publisher(Int32, "/tunnel_mode", 10)

        self.create_subscription(Image, "/video_frames", self.image_callback, 10)

        self.model = YOLO('yolov8n.pt')  # YOLOv8 nano 모델

        self.bridge = CvBridge()
        self.TUNNEL_SIZE_THRESHOLD = 100  # 픽셀 임계값

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            results = self.model(frame)

            bbox_array_msg = BoundingBox2DArray()
            bbox_array_msg.header.stamp = self.get_clock().now().to_msg()
            bbox_array_msg.header.frame_id = "camera_frame"

            tunnel_mode = 0

            for result in results:
                boxes = result.boxes.xywh.cpu().numpy()  # xywh format (center_x, center_y, w, h)
                classes = result.boxes.cls.cpu().numpy().astype(int)

                for bbox, cls_id in zip(boxes, classes):
                    cx, cy, w, h = bbox
                    if cls_id in [0, 1]:  # 라바콘, 드럼 등 주요 객체
                        box_msg = BoundingBox2D()
                        box_msg.x = float(cx - w / 2)  # 좌상단 x
                        box_msg.y = float(cy - h / 2)  # 좌상단 y
                        box_msg.w = float(w)
                        box_msg.h = float(h)
                        # id 필드는 메시지에 없으면 생략 가능
                        bbox_array_msg.boxes.append(box_msg)

                    elif cls_id == 2:  # 터널 감지
                        if w > self.TUNNEL_SIZE_THRESHOLD and h > self.TUNNEL_SIZE_THRESHOLD:
                            tunnel_mode = 1

            if len(bbox_array_msg.boxes) > 0:
                self.bbox_pub.publish(bbox_array_msg)

            self.tunnel_pub.publish(Int32(data=tunnel_mode))

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main():
    rclpy.init()
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
