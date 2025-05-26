# !/usr/bin/env python3
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
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from collections import defaultdict
import time

# 하이퍼파라미터
LAVA_CONE_WIDTH_THRESHOLD = 50  # 라바콘 바운딩 박스 최소 너비 (픽셀)
LAVA_CONE_HEIGHT_THRESHOLD = 50  # 라바콘 바운딩 박스 최소 높이 (픽셀)

DRUM_WIDTH_THRESHOLD = 80  # 드럼 바운딩 박스 최소 너비 (픽셀)
DRUM_HEIGHT_THRESHOLD = 80  # 드럼 바운딩 박스 최소 높이 (픽셀)

TUNNEL_WIDTH_THRESHOLD = 100  # 터널 바운딩 박스 최소 너비 (픽셀)
TUNNEL_HEIGHT_THRESHOLD = 100  # 터널 바운딩 박스 최소 높이 (픽셀)

# CAM & LiDAR 설정
CAMERA_FOV = 78  # 카메라 화각 (도)
LIDAR_FOV = 360  # LiDAR 화각 (도)
LIDAR_RANGE = 10  # LiDAR 최대 탐색 거리 (m)

# LiDAR 클러스터링 설정
DBSCAN_EPS = 0.5  # DBSCAN 거리 기준 (m)
DBSCAN_MIN_SAMPLES = 5  # DBSCAN 클러스터 최소 포인트 수

# 물체와 라이다 높이 설정
LIDAR_HEIGHT = 0.16  # 16cm
OBJECT_HEIGHT = 0.84  # 84cm


class ObstacleDetection(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # 카메라 내부 파라미터
        self.K = np.array([
            [640, 0, 320],   # fx, cx
            [0, 640, 320],   # fy, cy
            [0, 0, 1]
        ])

        # 외부 파라미터: 라이다와 카메라 간의 변환
        self.T = np.array([
            0.04,   # 라이다가 4cm 앞
            0.0,    # 좌우 차이 없음
            -0.06   # 라이다가 6cm 아래
        ]).reshape(3, 1)

        # 라이다가 카메라 기준 90도 회전된 상태
        # 카메라: x(오른쪽), y(아래), z(전방)
        # 라이다: x(후방), y(왼쪽), z(위)
        self.R = np.array([
            [-1, 0, 0],  # x축 방향 반전 (카메라 전방 -> 라이다 후방)
            [0, 0, 1],   # 카메라 z축 -> 라이다 y축
            [0, 1, 0]    # 카메라 y축 -> 라이다 z축
        ])


        # 객체 추적을 위한 칼만 필터 딕셔너리 추가
        self.kalman_filters = defaultdict(self.create_kalman_filter)
        self.last_detection_time = defaultdict(float)
        self.tracking_timeout = 1.0  # 1초 이상 감지되지 않으면 트래킹 중단

        # 퍼블리셔
        self.object_info_pub = self.create_publisher(String, "/object_info", 10)
        self.tunnel_info_pub = self.create_publisher(String, "/tunnel_info", 10)

        # 섭스크라이버
        self.create_subscription(Image, "/video_frames", self.image_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)

        # YOLO 모델 로드
        self.model = YOLO('../weights/YOLO_0216.pt')

        # 데이터 저장 변수
        self.bridge = CvBridge()
        self.img_bgr = None
        self.lidar_points = None
        self.filtered_points = None

        # 터널 모드 플래그
        self.tunnel_mode = False

    def create_kalman_filter(self):
        """칼만 필터 생성"""
        kf = KalmanFilter(dim_x=4, dim_z=2)  # 상태: [x, y, vx, vy], 측정: [x, y]
        kf.x = np.zeros(4)
        kf.F = np.array([[1., 0., 1., 0.],
                         [0., 1., 0., 1.],
                         [0., 0., 1., 0.],
                         [0., 0., 0., 1.]])  # 상태 전이 행렬
        kf.H = np.array([[1., 0., 0., 0.],
                         [0., 1., 0., 0.]])  # 측정 행렬
        kf.R = np.array([[0.1, 0.],
                         [0., 0.1]])  # 측정 노이즈
        kf.P *= 1000.  # 초기 불확실성
        kf.Q = Q_discrete_white_noise(dim=4, dt=0.1, var=0.1)
        return kf

    def image_callback(self, msg):
        """카메라 이미지 수신"""
        self.get_logger().info('Receiving video frame')
        current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.img_bgr = current_frame.copy()
        self.process_detections()

    def lidar_callback(self, msg):
        """2D LiDAR 데이터를 (x, y) 좌표로 변환"""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # LiDAR 포인트를 (x, y) 좌표로 변환
        lidar_points = np.array([ranges * np.cos(angles), ranges * np.sin(angles)]).T
        self.lidar_points = lidar_points

        # 간단한 필터링: 최대 거리 범위 내의 포인트만 선택
        self.filtered_points = lidar_points[ranges < LIDAR_RANGE]

    def cluster_lidar_points(self, points):
        """DBSCAN을 사용한 라이다 포인트 클러스터링"""
        if len(points) < DBSCAN_MIN_SAMPLES:
            return None, None

        clustering = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit(points)
        labels = clustering.labels_

        unique_labels = np.unique(labels)
        if len(unique_labels) == 1 and unique_labels[0] == -1:
            return None, None

        largest_cluster_label = max(unique_labels, key=lambda x: sum(labels == x) if x != -1 else -1)
        if largest_cluster_label == -1:
            return None, None

        cluster_points = points[labels == largest_cluster_label]
        cluster_center = np.mean(cluster_points, axis=0)

        return cluster_center, cluster_points

    def scan_lidar_for_object(self, bbox_x, bbox_y, bbox_height, bbox_width, label):
        """개선된 물체 감지 함수 - 라이다 높이에 해당하는 지점 사용"""
        if self.lidar_points is None or len(self.lidar_points) == 0:
            return

        # 바운딩 박스에서 라이다 높이에 해당하는 y좌표 계산
        height_ratio = LIDAR_HEIGHT / OBJECT_HEIGHT
        lidar_height_pixel_offset = int(bbox_height * (1 - height_ratio))

        # 라이다 높이에 해당하는 이미지 좌표 계산
        lidar_point_x = bbox_x
        lidar_point_y = bbox_y + lidar_height_pixel_offset  # bbox_y는 박스 상단

        # 이미지 좌표를 정규화된 카메라 좌표로 변환
        normalized_x = (lidar_point_x - self.K[0, 2]) / self.K[0, 0]
        normalized_y = (lidar_point_y - self.K[1, 2]) / self.K[1, 1]

        # 카메라 좌표계에서의 시선 벡터
        ray = np.array([normalized_x, normalized_y, 1]).reshape(3, 1)

        # 라이다 좌표계로 변환
        ray_lidar = self.R.T @ (ray - self.T)

        # 라이다 평면에서의 각도 계산
        angle = np.arctan2(ray_lidar[1], ray_lidar[0])

        # 바운딩 박스 크기에 기반한 검색 범위 설정
        angle_threshold = np.radians(max(5, min(30, bbox_width / 10)))

        # 예상되는 거리 범위 설정
        estimated_distance = (OBJECT_HEIGHT / normalized_y) if abs(normalized_y) > 0.1 else 5.0
        distance_threshold = estimated_distance * 0.3

        # 관심 영역의 라이다 포인트 선택
        lidar_angles = np.arctan2(self.lidar_points[:, 1], self.lidar_points[:, 0])
        distances = np.sqrt(np.sum(self.lidar_points ** 2, axis=1))

        # 각도와 거리 조건을 모두 만족하는 포인트 선택
        angle_diff = np.abs(lidar_angles - angle)
        distance_diff = np.abs(distances - estimated_distance)

        selected_points = self.lidar_points[
            (angle_diff < angle_threshold) &
            (distance_diff < distance_threshold)
            ]

        if len(selected_points) == 0:
            self.get_logger().warn(f"No LiDAR points found for {label} at expected location")
            return

        # 디버깅을 위한 시각화
        if self.img_bgr is not None:
            # 라이다 높이에 해당하는 점 표시 (빨간색)
            cv2.circle(self.img_bgr, (int(lidar_point_x), int(lidar_point_y)), 5, (0, 0, 255), -1)
            # 바운딩 박스 중심점 표시 (파란색)
            cv2.circle(self.img_bgr, (int(bbox_x), int(bbox_y)), 5, (255, 0, 0), -1)

        # 클러스터링 수행
        cluster_center, cluster_points = self.cluster_lidar_points(selected_points)
        if cluster_center is None:
            return

        # 칼만 필터 업데이트
        current_time = time.time()
        if current_time - self.last_detection_time[label] > self.tracking_timeout:
            self.kalman_filters[label] = self.create_kalman_filter()

        kf = self.kalman_filters[label]
        kf.predict()
        kf.update(cluster_center[:2])

        self.last_detection_time[label] = current_time

        # 필터링된 위치 얻기
        filtered_position = kf.x[:2]
        velocity = kf.x[2:]

        # 거리 계산 (높이 차이 고려)
        planar_distance = np.sqrt(np.sum(filtered_position ** 2))
        speed = np.sqrt(np.sum(velocity ** 2))

        # 로그 출력
        self.get_logger().info(
            f"Object {label} detected:"
            f"\nPosition (x,y): ({filtered_position[0]:.2f}, {filtered_position[1]:.2f})"
            f"\nPlanar Distance: {planar_distance:.2f}m"
            f"\nSpeed: {speed:.2f}m/s"
        )

        # 객체 정보 퍼블리시
        self.object_info_pub.publish(String(data=f"object,{label},"
                                                 f"{filtered_position[0]:.2f},"
                                                 f"{filtered_position[1]:.2f},"
                                                 f"{planar_distance:.2f},"
                                                 f"{speed:.2f}"))

    def process_detections(self):
        """YOLO 탐지 후 bbox 시각화 및 객체별 LiDAR 처리"""
        if self.img_bgr is not None:
            results = self.model(self.img_bgr)
            annotated_image = self.img_bgr.copy()
            tunnel_detected = False
            left_wall, right_wall = None, None

            for box in results[0].boxes:
                xyxy = box.xyxy[0]
                x1, y1, x2, y2 = map(int, xyxy)
                bbox_width = x2 - x1
                bbox_height = y2 - y1
                bbox_center_x = (x1 + x2) // 2
                bbox_center_y = (y1 + y2) // 2

                label = int(box.cls[0].item())
                conf = box.conf[0].item()

                # 라벨 텍스트 설정
                label_text = ["Lava Cone", "Drum", "Tunnel"][label] if label < 3 else f"Label {label}"

                # 바운딩 박스 및 라벨 시각화
                cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated_image, f"{label_text} {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # 조건에 따라 LiDAR 처리 수행
                if label == 0 and bbox_width > LAVA_CONE_WIDTH_THRESHOLD and bbox_height > LAVA_CONE_HEIGHT_THRESHOLD:
                    self.scan_lidar_for_object(bbox_center_x, y1, bbox_height, bbox_width, label)
                elif label == 1 and bbox_width > DRUM_WIDTH_THRESHOLD and bbox_height > DRUM_HEIGHT_THRESHOLD:
                    self.scan_lidar_for_object(bbox_center_x, y1, bbox_height, bbox_width, label)
                elif label == 2 and bbox_width > TUNNEL_WIDTH_THRESHOLD and bbox_height > TUNNEL_HEIGHT_THRESHOLD:
                    tunnel_detected = True
                    left_wall, right_wall = self.estimate_tunnel_walls()

            # 터널 정보 퍼블리시
            self.tunnel_info_pub.publish(String(data=f"tunnel,{int(tunnel_detected)},{left_wall},{right_wall}"))

            # 이미지 표시
            cv2.imshow("camera", annotated_image)
            cv2.waitKey(1)

    def estimate_tunnel_walls(self):
        """LiDAR 데이터에서 좌우 벽의 위치 추정"""
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