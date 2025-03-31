#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rclpy
import torch
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from math import cos, sin

# ===========================
# PyTorch YOLOv8 모델 로드 클래스
# ===========================
class TorchYOLOv8:
    def __init__(self, model_path):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path).to(self.device)
        self.model.eval()
    
    def preprocess(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (640, 640))
        img = torch.from_numpy(img).float() / 255.0
        img = img.permute(2, 0, 1).unsqueeze(0)  # (H, W, C) -> (1, C, H, W)
        return img.to(self.device)
    
    def detect(self, img):
        img_tensor = self.preprocess(img)
        with torch.no_grad():
            results = self.model(img_tensor)
        return results.xyxy[0].cpu().numpy()  # (x1, y1, x2, y2, conf, cls)

# ===========================
# ROS2 장애물 탐지 노드
# ===========================
class ObstacleDetection(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # 퍼블리셔
        self.object_info_pub = self.create_publisher(String, "/object_info", 10)
        self.tunnel_info_pub = self.create_publisher(String, "/tunnel_info", 10)

        # 섭스크라이버
        self.create_subscription(Image, "/image_jpeg", self.image_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)

        # PyTorch YOLOv8 모델 로드
        self.model = TorchYOLOv8('/home/antel/2025IEVE_1of5/2025IEVE/YOLO_ws/weights/yolov8.pt')
        
        # 데이터 저장 변수
        self.bridge = CvBridge()
        self.img_bgr = None
        self.lidar_points = None
    
    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if self.img_bgr is not None:
            self.process_detections()
    
    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        self.lidar_points = np.array([ranges * np.cos(angles), ranges * np.sin(angles)]).T
    
    def process_detections(self):
        detections = self.model.detect(self.img_bgr)
        
        for det in detections:
            x1, y1, x2, y2, conf, label = det
            label = int(label)
            cv2.rectangle(self.img_bgr, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(self.img_bgr, f"{label}: {conf:.2f}", (int(x1), int(y1) - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        self.visualize_lidar()
    
    def visualize_lidar(self):
        if self.lidar_points is None:
            return
        
        for point in self.lidar_points:
            x, y = int(point[0] * 10 + 320), int(480 - point[1] * 10)
            if 0 <= x < 640 and 0 <= y < 480:
                cv2.circle(self.img_bgr, (x, y), 2, (0, 0, 255), -1)
        
        cv2.imshow("YOLO & LiDAR", self.img_bgr)
        cv2.waitKey(1)

# ===========================
# 메인 실행
# ===========================
def main():
    rclpy.init()
    detector = ObstacleDetection()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
