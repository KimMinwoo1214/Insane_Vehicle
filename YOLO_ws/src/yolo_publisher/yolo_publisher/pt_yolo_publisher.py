#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32MultiArray
from cv_bridge import CvBridge

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # 퍼블리셔
        self.bbox_pub = self.create_publisher(Float32MultiArray, "/bbox_centers", 10)
        self.tunnel_pub = self.create_publisher(Int32, "/tunnel_mode", 10)
        
        # 섭스크라이버
        self.create_subscription(Image, "/video_frames", self.image_callback, 10)
        
        # YOLOv11 모델 로드 - 경로는 실제 모델 위치에 맞게 수정 필요
        try:
            self.model = YOLO('yolov11n.pt')  # 기본 YOLOv11 nano 모델
            # 또는 custom trained 모델인 경우:
            # self.model = YOLO('../weights/YOLO_0216.pt')
        except Exception as e:
            self.get_logger().error(f'모델 로드 실패: {str(e)}')
            raise

        self.bridge = CvBridge()
        self.TUNNEL_SIZE_THRESHOLD = 100  # 픽셀 단위

    def image_callback(self, msg):
        """카메라 이미지 처리"""
        try:
            current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # YOLOv11에서는 conf와 iou 임계값을 명시적으로 설정하는 것이 권장됨
            results = self.model.predict(
                source=current_frame,
                conf=0.25,  # confidence threshold
                iou=0.45,   # NMS IoU threshold
                verbose=False
            )
            
            centers = []
            tunnel_mode = 0

            # YOLOv11의 결과 처리
            if results and len(results) > 0:
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        # YOLOv11에서는 xyxy가 기본적으로 numpy array로 반환됨
                        xyxy = box.xyxy[0] if isinstance(box.xyxy, list) else box.xyxy
                        x1, y1, x2, y2 = map(int, xyxy)
                        label = int(box.cls) if isinstance(box.cls, (int, float)) else int(box.cls[0])
                        
                        # 클래스 0, 1인 경우 중심점 저장
                        if label in [0, 1]:
                            center_x = (x1 + x2) / 2
                            center_y = (y1 + y2) / 2
                            centers.extend([center_x, center_y])
                        
                        # 클래스 2(터널)이고 크기가 임계값을 넘는 경우
                        elif label == 2:
                            width = x2 - x1
                            height = y2 - y1
                            if width > self.TUNNEL_SIZE_THRESHOLD and height > self.TUNNEL_SIZE_THRESHOLD:
                                tunnel_mode = 1

            # bbox 중심점 발행
            if centers:
                msg = Float32MultiArray()
                msg.data = centers
                self.bbox_pub.publish(msg)

            # 터널 모드 발행
            self.tunnel_pub.publish(Int32(data=tunnel_mode))

        except Exception as e:
            self.get_logger().error(f'이미지 처리 중 에러 발생: {str(e)}')

def main():
    rclpy.init()
    detector = ObjectDetection()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()