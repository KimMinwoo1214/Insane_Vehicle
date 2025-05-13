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

DRUM_WIDTH_THRESHOLD = 80        # 드럼 바운딩 박스 최소 너비 (픽셀)
DRUM_HEIGHT_THRESHOLD = 80       # 드럼 바운딩 박스 최소 높이 (픽셀)

TUNNEL_WIDTH_THRESHOLD = 100     # 터널 바운딩 박스 최소 너비 (픽셀)
TUNNEL_HEIGHT_THRESHOLD = 100    # 터널 바운딩 박스 최소 높이 (픽셀)

# 카메라 내부 행렬
K = np.array([[700, 0, 320],
              [0, 700, 240],
              [0, 0, 1]])

# 카메라-라이다 높이 차이 (m)
CAMERA_LIDAR_HEIGHT_DIFF = 0.05

# CAM & LiDAR 설정
CAMERA_FOV = 78   # 카메라 화각 (도)
LIDAR_FOV = 360   # LiDAR 화각 (도)
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
        self.create_subscription(Image, "/video_frames", self.image_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)

        # YOLO 모델 로드
        self.model = YOLO('/home/antel/2025IEVE_1of5/2025IEVE/YOLO_ws/weights/YOLO_0216.pt')  

        # 데이터 저장 변수
        self.bridge = CvBridge()
        self.img_bgr = None
        self.lidar_points = None
        self.filtered_points = None

        # 터널 모드 플래그
        self.tunnel_mode = False

    def image_callback(self, msg):
        """ 카메라 이미지 수신 후 YOLO 감지 및 bbox 시각화 """
        self.get_logger().info('Receiving video frame')
 
        # ROS Image 메시지를 OpenCV 이미지로 변환 (BGR 형식)
        current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
        # 수신 프레임 저장
        self.img_bgr = current_frame.copy()
        
        # YOLO 감지 및 bbox 시각화 처리
        self.process_detections()

    def lidar_callback(self, msg):
        """ 2D LiDAR 데이터를 (x, y) 좌표로 변환 후 ROI 필터 적용 """
        ranges = np.array(msg.ranges)  # LiDAR 거리 값
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))  # 각도 값

        # LiDAR 포인트를 (x, y) 좌표로 변환
        lidar_points = np.array([ranges * np.cos(angles), ranges * np.sin(angles)]).T
        
        # LiDAR 포인트를 클래스 변수에 저장
        self.lidar_points = lidar_points

        # 간단한 필터링 로직: 최대 거리 범위 내의 포인트만 필터링
        max_range = LIDAR_RANGE  # 최대 탐색 거리
        self.filtered_points = lidar_points[ranges < max_range]  # 최대 거리 내의 포인트만 필터링

        # 디버깅을 위한 로그
        self.get_logger().info(f"Total LiDAR points: {len(lidar_points)}")
        self.get_logger().info(f"Filtered LiDAR points: {len(self.filtered_points)}")
        if len(self.filtered_points) == 0:
            self.get_logger().warn("No LiDAR points available after filtering!")

    def process_detections(self):
        """ YOLO 탐지 후 bounding box 시각화 및 객체별 LiDAR 처리 및 퍼블리시 """
        if self.img_bgr is not None:
            # YOLO 모델을 사용하여 감지 실행
            results = self.model(self.img_bgr)
            
            # 원본 이미지를 복사하여 bbox 그리기
            annotated_image = self.img_bgr.copy()
            tunnel_detected = False
            left_wall, right_wall = None, None

            for box in results[0].boxes:
                # xyxy 좌표로 bounding box 추출
                xyxy = box.xyxy[0]
                x1 = int(xyxy[0].item())
                y1 = int(xyxy[1].item())
                x2 = int(xyxy[2].item())
                y2 = int(xyxy[3].item())
                bbox_width = x2 - x1
                bbox_height = y2 - y1

                # bounding box 중앙 x좌표 (LiDAR 각도 산출용)
                bbox_center_x = int((x1 + x2) / 2)
                label = int(box.cls[0].item())
                conf = box.conf[0].item()

                # 라벨 별 텍스트 설정 (필요시 수정)
                if label == 0:
                    label_text = "Lava Cone"
                elif label == 1:
                    label_text = "Drum"
                elif label == 2:
                    label_text = "Tunnel"
                else:
                    label_text = f"Label {label}"

                # bounding box 및 라벨, 신뢰도 시각화
                cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated_image, f"{label_text} {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # 조건에 따라 LiDAR 처리 수행
                if label == 0 and bbox_width > LAVA_CONE_WIDTH_THRESHOLD and bbox_height > LAVA_CONE_HEIGHT_THRESHOLD:
                    self.scan_lidar_for_object(bbox_center_x, label)
                elif label == 1 and bbox_width > DRUM_WIDTH_THRESHOLD and bbox_height > DRUM_HEIGHT_THRESHOLD:
                    self.scan_lidar_for_object(bbox_center_x, label)
                elif label == 2 and bbox_width > TUNNEL_WIDTH_THRESHOLD and bbox_height > TUNNEL_HEIGHT_THRESHOLD:
                    tunnel_detected = True
                    left_wall, right_wall = self.estimate_tunnel_walls()

            # 터널 정보 퍼블리시
            self.tunnel_info_pub.publish(String(data=f"tunnel,{int(tunnel_detected)},{left_wall},{right_wall}"))

            # annotated_image를 윈도우에 출력
            cv2.imshow("camera", annotated_image)
            cv2.waitKey(1)

    def scan_lidar_for_object(self, bbox_x, label):
        """ YOLO 바운딩 박스 위치를 기반으로 LiDAR 스캔 수행 및 객체 정보 퍼블리시 """
        if self.lidar_points is None or len(self.lidar_points) == 0:
            self.get_logger().warn(f"No LiDAR points available for object {label}")
            return

        # LiDAR 데이터 전체 사용 (필터링 없음)
        selected_points = self.lidar_points

        # 선택된 LiDAR 포인트가 없다면
        if selected_points.shape[0] == 0:
            self.get_logger().warn(f"No LiDAR points selected for object {label}")
        else:
            self.get_logger().info(f"Selected {selected_points.shape[0]} LiDAR points for object {label}")

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

