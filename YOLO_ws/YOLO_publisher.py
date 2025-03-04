#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
import torch
from ultralytics import YOLO
from sensor_msgs.msg import LaserScan, CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import time
from scipy.spatial import KDTree
from sklearn.cluster import DBSCAN 
from math import cos, sin, sqrt

# YOLO 학습된 클래스
CON = 0  # 라바콘
DRUM = 1  # PE드럼

# 터널 감지 기준
TUNNEL_POINT_THRESHOLD = 10  # 좌우 각각 최소 감지 포인트 개수
TUNNEL_WIDTH_MIN = 0.5       # 좌우 벽 간 최소 거리 (m)
TUNNEL_WIDTH_MAX = 4.0       # 좌우 벽 간 최대 거리 (m)
DBSCAN_EPS = 0.5             # DBSCAN 거리 기준 (m)
DBSCAN_MIN_SAMPLES = 5       # DBSCAN 클러스터 최소 포인트 수

# 하이퍼파라미터: LiDAR가 bbox 높이의 몇 % 높이에 있는지 (라바콘 기준)
HEIGHT_RATIO = 0.3  # 0.0 ~ 1.0

class ConeDrumDetection:
    def __init__(self):
        rospy.init_node('cone_drum_detector', anonymous=True)

        # ROS 파라미터에서 높이 비율 조정 가능
        self.height_ratio = rospy.get_param("~height_ratio", HEIGHT_RATIO)

        # 퍼블리셔
        self.visualization_publish = rospy.Publisher("/yolo_viz", MarkerArray, queue_size=10)  # RViz용 마커
        self.image_pub = rospy.Publisher("/yolo_debug", CompressedImage, queue_size=10)  # SSH 디버깅용 이미지
        self.object_info_pub = rospy.Publisher("/object_info", String, queue_size=10)  # 플래닝이 쓸 토픽

        # 섭스크라이버
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback, queue_size=10)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        # YOLO 모델 로드
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
        roi_mask = (self.lidar_points[:, 0] > 0.5) & (self.lidar_points[:, 0] < 10.0) & \
                   (self.lidar_points[:, 1] > -3.0) & (self.lidar_points[:, 1] < 3.0)
        self.filtered_points = self.lidar_points[roi_mask]

        # LiDAR만으로 터널 감지
        self.detect_tunnel()

    def process_detections(self):
        """ YOLO 탐지 수행 후 2D LiDAR와 매칭 """
        if self.img_bgr is not None:
            self.frame_counter += 1
            if self.frame_counter % self.frame_interval == 0:
                res = self.model.predict(self.img_bgr, stream=True)

                markers = MarkerArray()

                if len(res[0].boxes) > 0:
                    # LiDAR 데이터 예외 처리
                    if self.filtered_points is None or len(self.filtered_points) == 0:
                        rospy.logwarn("No valid LiDAR points available.")
                        return  # LiDAR 데이터가 없으면 탐지 수행 X

                    for i, box in enumerate(res[0].boxes):
                        cls = int(box.cls.item())  # YOLO가 예측한 클래스 ID

                        if cls == CON:
                            self.create_marker(box, i, "con", (1.0, 0.5, 0.0))
                        elif cls == DRUM:
                            self.create_marker(box, i, "drum", (1.0, 0.0, 0.0))

                self.visualization_publish.publish(markers)

                # SSH 사용 시 토픽으로 YOLO 탐지 결과 퍼블리시
                msg = self.bridge.cv2_to_compressed_imgmsg(res[0].plot())
                self.image_pub.publish(msg)

    def detect_tunnel(self):
        """ ✅ LiDAR 데이터만으로 터널 감지 후 퍼블리시 """
        if self.filtered_points is None or len(self.filtered_points) == 0:
            return

        # 좌우로 LiDAR 포인트 나누기
        left_points = self.filtered_points[self.filtered_points[:, 1] > 0]
        right_points = self.filtered_points[self.filtered_points[:, 1] < 0]

        # 좌우 LiDAR 포인트 개수 조건
        if len(left_points) >= TUNNEL_POINT_THRESHOLD and len(right_points) >= TUNNEL_POINT_THRESHOLD:
            left_x_mean = np.mean(left_points[:, 0])
            right_x_mean = np.mean(right_points[:, 0])
            tunnel_width = abs(left_x_mean - right_x_mean)

            # 터널 폭 조건
            if TUNNEL_WIDTH_MIN <= tunnel_width <= TUNNEL_WIDTH_MAX:
                tunnel_x = (left_x_mean + right_x_mean) / 2
                tunnel_y = (np.mean(left_points[:, 1]) + np.mean(right_points[:, 1])) / 2
                distance = sqrt(tunnel_x**2 + tunnel_y**2)

                self.object_info_pub.publish(f"tunnel,{tunnel_x:.2f},{tunnel_y:.2f},{distance:.2f}")

                # DBSCAN
                dbscan = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES)
                labels = dbscan.fit_predict(self.filtered_points)

                if np.any(labels != -1):  # 클러스터가 존재하면 터널로 확정
                    self.object_info_pub.publish(f"tunnel_confirmed,{tunnel_x:.2f},{tunnel_y:.2f},{distance:.2f}")

if __name__ == '__main__':
    detector = ConeDrumDetection()
    rospy.spin()
