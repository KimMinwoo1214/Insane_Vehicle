#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rclpy
from rclpy.node import Node
import numpy as np
import torch
from ultralytics import YOLO
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from scipy.spatial import KDTree
from sklearn.cluster import DBSCAN
from math import cos, sin, sqrt

# 하이퍼파라미터
LAVA_CONE_WIDTH_THRESHOLD = 50   # 라바콘 바운딩 박스 최소 너비 (픽셀)
LAVA_CONE_HEIGHT_THRESHOLD = 50  # 라바콘 바운딩 박스 최소 높이 (픽셀)

DRUM_WIDTH_THRESHOLD = 80   # 드럼 바운딩 박스 최소 너비 (픽셀)
DRUM_HEIGHT_THRESHOLD = 80  # 드럼 바운딩 박스 최소 높이 (픽셀)

TUNNEL_WIDTH_THRESHOLD = 100   # 터널 바운딩 박스 최소 너비 (픽셀)
TUNNEL_HEIGHT_THRESHOLD = 100  # 터널 바운딩 박스 최소 높이 (픽셀)

# 카메라 내부 행렬
K = np.array([[700, 0, 320],
              [0, 700, 240],
              [0, 0, 1]])

# 카메라-라이다 높이 차이 (m)
CAMERA_LIDAR_HEIGHT_DIFF = 0.3

# CAM & LiDAR 설정
CAMERA_FOV = 70   # 카메라 화각 (도)
LIDAR_FOV = 270   # LiDAR 화각 (도)
LIDAR_RANGE = 10  # LiDAR 최대 탐색 거리 (m)

# LiDAR 클러스터링 설정
DBSCAN_EPS = 0.5            # DBSCAN 거리 기준 (m)
DBSCAN_MIN_SAMPLES = 5      # DBSCAN 클러스터 최소 포인트 수

class ObstacleDetection(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # 퍼블리셔
        self.object_info_pub = self.create_publisher(String, "/object_info", 10)
        self.tunnel_info_pub = self.create_publisher(String, "/tunnel_info", 10)

        # 섭스크라이버
        self.create_subscription(Image, "/image_jpeg", self.image_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)

        # YOLO 모델 로드
        self.model = YOLO('/path/to/yolo_weights.pt')  

        # 데이터 저장 변수
        self.bridge = CvBridge()
        self.img_bgr = None
        self.lidar_points = None
        self.filtered_points = None

        # 터널 모드 플래그
        self.tunnel_mode = False

    def image_callback(self, msg):
        """ 카메라 이미지 수신 후 YOLO 감지 수행 """
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def lidar_callback(self, msg):
        """ 2D LiDAR 데이터를 (x, y) 좌표로 변환 후 ROI 필터 적용 """
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        self.lidar_points = np.array([ranges * np.cos(angles), ranges * np.sin(angles)]).T

    def process_detections(self):
        """ YOLO 탐지 후 객체별 LiDAR 처리 및 퍼블리시 """
        if self.img_bgr is not None:
            res = self.model.predict(self.img_bgr, stream=True)

            tunnel_detected = False
            left_wall, right_wall = None, None

            for box in res[0].boxes:
                bbox_x = box.xywh[0, 0].item()
                bbox_y = box.xywh[0, 1].item()
                bbox_width = box.xywh[0, 2].item()
                bbox_height = box.xywh[0, 3].item()
                label = int(box.cls[0].item())  

                # 라바콘 (label 0) - LiDAR 스캔 후 퍼블리시
                if label == 0 and bbox_width > LAVA_CONE_WIDTH_THRESHOLD and bbox_height > LAVA_CONE_HEIGHT_THRESHOLD:
                    self.scan_lidar_for_object(bbox_x, label)

                # 드럼 (label 1) - LiDAR 스캔 후 퍼블리시
                if label == 1 and bbox_width > DRUM_WIDTH_THRESHOLD and bbox_height > DRUM_HEIGHT_THRESHOLD:
                    self.scan_lidar_for_object(bbox_x, label)

                # 터널 (label 2) - 크기 조건 만족 시 LiDAR 벽 위치 계산 후 퍼블리시
                if label == 2 and bbox_width > TUNNEL_WIDTH_THRESHOLD and bbox_height > TUNNEL_HEIGHT_THRESHOLD:
                    tunnel_detected = True
                    left_wall, right_wall = self.estimate_tunnel_walls()

            # 터널 정보 퍼블리시
            self.tunnel_info_pub.publish(String(data=f"tunnel,{int(tunnel_detected)},{left_wall},{right_wall}"))

    def scan_lidar_for_object(self, bbox_x, label):
        """ YOLO 바운딩 박스 위치를 기반으로 LiDAR 스캔 수행 및 객체 정보 퍼블리시 """
        if self.filtered_points is None or len(self.filtered_points) == 0:
            return

        # LiDAR 탐색 각도 변환
        angle_ratio = bbox_x / 640
        lidar_angle = (angle_ratio * LIDAR_FOV) - (LIDAR_FOV / 2)
        angle_min = np.deg2rad(lidar_angle - 10)
        angle_max = np.deg2rad(lidar_angle + 10)

        # 특정 각도 범위의 LiDAR 데이터만 필터링
        angles = np.arctan2(self.filtered_points[:, 1], self.filtered_points[:, 0])
        mask = (angles > angle_min) & (angles < angle_max)
        selected_points = self.filtered_points[mask]

        if selected_points.shape[0] > 0:
            center_x = np.mean(selected_points[:, 0])
            center_y = np.mean(selected_points[:, 1])
            distance = sqrt(center_x**2 + center_y**2)
            self.object_info_pub.publish(String(data=f"object,{label},{center_x:.2f},{center_y:.2f},{distance:.2f}"))

    def estimate_tunnel_walls(self):
        """ LiDAR 데이터에서 좌우 벽의 위치 추정 """
        if self.filtered_points is None or len(self.filtered_points) == 0:
            return None, None

        left_wall = np.min(self.filtered_points[:, 1])
        right_wall = np.max(self.filtered_points[:, 1])

        return round(left_wall, 2), round(right_wall, 2)

def main():
    rclpy.init()
    detector = ObstacleDetection()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

