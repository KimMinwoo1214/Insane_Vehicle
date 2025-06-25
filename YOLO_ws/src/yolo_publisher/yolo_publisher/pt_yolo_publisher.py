#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from custom_msgs.msg import BoundingBox2D, BoundingBox2DArray
from std_msgs.msg import Header

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        self.bbox_pub = self.create_publisher(BoundingBox2DArray, "/yolo_bounding_boxes", 10)
        self.create_subscription(Image, "/video_frames", self.image_callback, 10)
        
        try:
            self.model = YOLO('../weights/YOLO_0216.pt')
        except Exception as e:
            self.get_logger().error(f'모델 로드 실패: {str(e)}')
            raise

        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            results = self.model.predict(
                source=current_frame,
                conf=0.25,
                iou=0.45,
                verbose=False
            )

            bbox_array = BoundingBox2DArray()
            bbox_array.header = Header()
            bbox_array.header.stamp = self.get_clock().now().to_msg()
            bbox_array.header.frame_id = "camera_frame"  # ← 필요시 실제 프레임으로 변경

            if results and len(results) > 0:
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        xyxy = box.xyxy.cpu().numpy().flatten()
                        x1, y1, x2, y2 = map(int, xyxy)

                        msg_box = BoundingBox2D()
                        msg_box.xmin = x1
                        msg_box.ymin = y1
                        msg_box.xmax = x2
                        msg_box.ymax = y2
                        bbox_array.boxes.append(msg_box)

            if bbox_array.boxes:
                self.bbox_pub.publish(bbox_array)

            # (옵션) 디버깅용 화면 표시
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

