#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rclpy
from rclpy.node import Node
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from scipy.spatial import KDTree
from sklearn.cluster import DBSCAN
from math import cos, sin, sqrt

# ===========================
# TensorRT YOLOv8 엔진 로드 클래스
# ===========================
class TrtYOLOv8:
    def __init__(self, engine_path):
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            runtime = trt.Runtime(self.logger)
            self.engine = runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()

        # 입력/출력 바인딩 설정
        self.input_idx = self.engine.get_binding_index("images")
        self.output_idx = self.engine.get_binding_index("output0")

        self.input_shape = self.engine.get_binding_shape(self.input_idx)
        self.output_shape = self.engine.get_binding_shape(self.output_idx)

        self.input_size = np.prod(self.input_shape) * np.dtype(np.float32).itemsize
        self.output_size = np.prod(self.output_shape) * np.dtype(np.float32).itemsize

        # GPU 메모리 할당
        self.d_input = cuda.mem_alloc(self.input_size)
        self.d_output = cuda.mem_alloc(self.output_size)

        self.stream = cuda.Stream()

    def preprocess(self, img):
        """ 이미지 전처리 (YOLOv8 TensorRT 형식으로 변환) """
        img = cv2.resize(img, (640, 640))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC → CHW 변환
        img = np.expand_dims(img, axis=0)   # 배치 차원 추가
        return np.ascontiguousarray(img)

    def detect(self, img):
        """ TensorRT YOLOv8 추론 실행 """
        img_input = self.preprocess(img)
        
        cuda.memcpy_htod_async(self.d_input, img_input, self.stream)
        self.context.execute_v2([self.d_input, self.d_output])
        output = np.empty(self.output_shape, dtype=np.float32)
        cuda.memcpy_dtoh_async(output, self.d_output, self.stream)
        self.stream.synchronize()
        
        return output  # (N, 6) 형식으로 반환됨: [x, y, w, h, conf, class]

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

        # YOLOv8 TensorRT 모델 로드
        self.model = TrtYOLOv8('#############경로 적기############')

        # 데이터 저장 변수
        self.bridge = CvBridge()
        self.img_bgr = None
        self.lidar_points = None
        self.filtered_points = None

    def image_callback(self, msg):
        """ 카메라 이미지 수신 후 YOLOv8 TensorRT 탐지 수행 """
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
            res = self.model.detect(self.img_bgr)

            tunnel_detected = False
            left_wall, right_wall = None, None

            for det in res:
                bbox_x, bbox_y, bbox_width, bbox_height, conf, label = det
                label = int(label)

                if label == 0 and bbox_width > 50 and bbox_height > 50:  # 라바콘
                    self.scan_lidar_for_object(bbox_x, label)
                if label == 1 and bbox_width > 80 and bbox_height > 80:  # 드럼
                    self.scan_lidar_for_object(bbox_x, label)
                if label == 2 and bbox_width > 100 and bbox_height > 100:  # 터널
                    tunnel_detected = True
                    left_wall, right_wall = self.estimate_tunnel_walls()

            self.tunnel_info_pub.publish(String(data=f"tunnel,{int(tunnel_detected)},{left_wall},{right_wall}"))

    def scan_lidar_for_object(self, bbox_x, label):
        """ YOLO 바운딩 박스 위치를 기반으로 LiDAR 스캔 수행 및 객체 정보 퍼블리시 """
        if self.filtered_points is None or len(self.filtered_points) == 0:
            return

        angle_ratio = bbox_x / 640
        lidar_angle = (angle_ratio * 270) - (270 / 2)
        angle_min = np.deg2rad(lidar_angle - 10)
        angle_max = np.deg2rad(lidar_angle + 10)

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
