#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
import torch
from ultralytics import YOLO
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import time
from scipy.spatial import KDTree
from sklearn.cluster import DBSCAN
from math import cos, sin, sqrt

# 하이퍼파라미터
BBOX_WIDTH_THRESHOLD = 50   # YOLO 바운딩 박스 가로 크기 최소값 (픽셀)
BBOX_HEIGHT_THRESHOLD = 50  # YOLO 바운딩 박스 세로 크기 최소값 (픽셀)
LIDAR_HEIGHT_OFFSET = 0.05  # LiDAR와 카메라 높이 차이 (m)

# CAM & LiDAR 설정
CAMERA_FOV = 70   # 카메라 화각 (도)
LIDAR_FOV = 270   # LiDAR 화각 (도)
LIDAR_RANGE = 10  # LiDAR 최대 탐색 거리 (m)

# LiDAR 클러스터링 설정
DBSCAN_EPS = 0.5            # DBSCAN 거리 기준 (m)
DBSCAN_MIN_SAMPLES = 5      # DBSCAN 클러스터 최소 포인트 수

# 터널 감지 설정
TUNNEL_MIN_WIDTH = 1.0  # 터널 최소 폭 (m)
TUNNEL_MAX_WIDTH = 4.0  # 터널 최대 폭 (m)
TUNNEL_MIN_POINTS = 10  # 좌우 각각 최소 감지 포인트 개수


class ObstacleDetection:
    def __init__(self):
        rospy.init_node('obstacle_detector', anonymous=True)

        # 퍼블리셔
        self.object_info_pub = rospy.Publisher("/object_info", String, queue_size=10)  # 플래닝용 장애물 정보
        self.image_pub = rospy.Publisher("/yolo_debug", CompressedImage, queue_size=10)  # YOLO 감지 시각화

        # 섭스크라이버
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback, queue_size=10)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        # YOLO 모델 로드 (장애물 감지용)
        self.model = YOLO('/home/user/YOLO_ws/weights/YOLO_0216.pt')  # 경로 수정 필요

        # 데이터 저장 변수
        self.bridge = CvBridge()
        self.img_bgr = None
        self.lidar_points = None
        self.filtered_points = None

        # YOLO 실행 간격 (프레임 간격 조정)
        self.frame_counter = 0
        self.frame_interval = 2  # 2 프레임마다 YOLO 실행

    def image_callback(self, msg):
        """ 카메라 이미지 수신 후 YOLO 감지 수행 """
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def lidar_callback(self, msg):
        """ 2D LiDAR 데이터를 (x, y) 좌표로 변환 후 ROI 필터 적용 """
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # 극좌표 → 직교좌표 변환
        self.lidar_points = np.array([ranges * np.cos(angles), ranges * np.sin(angles)]).T

        # ROI 필터 적용 (전방 10m 이내, 좌우 ±3m 범위)
        roi_mask = (self.lidar_points[:, 0] > 0.5) & (self.lidar_points[:, 0] < LIDAR_RANGE) & \
                   (self.lidar_points[:, 1] > -3.0) & (self.lidar_points[:, 1] < 3.0)
        self.filtered_points = self.lidar_points[roi_mask]

    def process_detections(self):
        """ YOLO 탐지 후 LiDAR 탐색 영역 변환 """
        if self.img_bgr is not None:
            self.frame_counter += 1
            if self.frame_counter % self.frame_interval == 0:
                res = self.model.predict(self.img_bgr, stream=True)

                if len(res[0].boxes) > 0:
                    for box in res[0].boxes:
                        bbox_x = box.xywh[0, 0].item()
                        bbox_y = box.xywh[0, 1].item()
                        bbox_width = box.xywh[0, 2].item()
                        bbox_height = box.xywh[0, 3].item()

                        # 바운딩 박스 크기 기준 만족 시 LiDAR 탐색 영역 변환
                        if bbox_width > BBOX_WIDTH_THRESHOLD and bbox_height > BBOX_HEIGHT_THRESHOLD:
                            self.map_bbox_to_lidar(bbox_x)

                # YOLO 탐지 결과 퍼블리시
                msg = self.bridge.cv2_to_compressed_imgmsg(res[0].plot())
                self.image_pub.publish(msg)

    def map_bbox_to_lidar(self, bbox_x):
        """ YOLO 바운딩 박스 x좌표 → LiDAR 탐색 각도로 변환 """
        x_ratio = bbox_x / 640  # 640은 이미지 가로 크기
        camera_angle = (x_ratio * CAMERA_FOV) - (CAMERA_FOV / 2)

        # LiDAR 기준 각도로 변환
        lidar_angle = ((camera_angle + (CAMERA_FOV / 2)) / CAMERA_FOV) * LIDAR_FOV - (LIDAR_FOV / 2)
        angle_min = np.deg2rad(lidar_angle - 10)  # 탐색 범위 ±10도
        angle_max = np.deg2rad(lidar_angle + 10)

        # LiDAR 필터링 (높이 차이 반영)
        if self.filtered_points is not None and len(self.filtered_points) > 0:
            angles = np.arctan2(self.filtered_points[:, 1], self.filtered_points[:, 0])
            mask = (angles > angle_min) & (angles < angle_max)
            selected_points = self.filtered_points[mask]

            # 장애물 탐지
            self.detect_obstacle_from_lidar(selected_points)

    def detect_obstacle_from_lidar(self, selected_points):
        """ LiDAR 포인트 클러스터링을 통한 장애물 탐지 (특정 탐색 범위) """
        if selected_points is None or len(selected_points) == 0:
            return

        # DBSCAN을 사용한 LiDAR 포인트 클러스터링
        dbscan = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES)
        labels = dbscan.fit_predict(selected_points)

        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:  # 노이즈 제거
                continue

            # 클러스터 중심 계산
            cluster_points = selected_points[labels == label]
            center_x = np.mean(cluster_points[:, 0])
            center_y = np.mean(cluster_points[:, 1])
            distance = sqrt(center_x**2 + center_y**2)

            # 장애물 정보 퍼블리시
            self.object_info_pub.publish(f"obstacle,{center_x:.2f},{center_y:.2f},{distance:.2f}")

    def detect_tunnel(self):
        """ LiDAR 데이터를 사용하여 터널 감지 및 플래닝에게 정보 전달 """
        ##### 차가 틀어져있을 때 헤딩값 가지고 보정하는 부분 필요할듯
        if self.filtered_points is None or len(self.filtered_points) == 0:
            return

        # 좌우 LiDAR 포인트 분리
        left_points = self.filtered_points[self.filtered_points[:, 1] > 0]
        right_points = self.filtered_points[self.filtered_points[:, 1] < 0]

        # 좌우 벽 감지 조건 (터널 내부로 판단)
        if len(left_points) >= TUNNEL_MIN_POINTS and len(right_points) >= TUNNEL_MIN_POINTS:
            left_x_mean = np.mean(left_points[:, 0])
            right_x_mean = np.mean(right_points[:, 0])
            tunnel_width = abs(left_x_mean - right_x_mean)

            # 터널 폭이 일정 범위 내에 있는지 확인
            if TUNNEL_MIN_WIDTH <= tunnel_width <= TUNNEL_MAX_WIDTH:
                tunnel_x = (left_x_mean + right_x_mean) / 2
                tunnel_y = (np.mean(left_points[:, 1]) + np.mean(right_points[:, 1])) / 2
                distance = sqrt(tunnel_x**2 + tunnel_y**2)

                # 플래닝에게 터널 정보 전달
                tunnel_msg = f"mode:tunnel,{left_x_mean:.2f},{right_x_mean:.2f}"
                self.object_info_pub.publish(tunnel_msg)


if __name__ == '__main__':
    detector = ObstacleDetection()
    rospy.spin()
