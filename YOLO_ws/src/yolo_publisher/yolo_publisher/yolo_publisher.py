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
        
        return output  # (N, 6) 형식: [x, y, w, h, conf, class]

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

    def image_callback(self, msg):
        """ 카메라 이미지 수신 후 YOLOv8 TensorRT 탐지 수행 """
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # 탐지 및 시각화 실행
        if self.img_bgr is not None:
            self.process_detections()

    def lidar_callback(self, msg):
        """ 2D LiDAR 데이터를 (x, y) 좌표로 변환 후 ROI 필터 적용 """
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        lidar_points = np.array([ranges * np.cos(angles), ranges * np.sin(angles)]).T

        # 높이 차이 보정 (카메라가 LiDAR보다 5cm 위에 있음)
        lidar_points[:, 1] -= 0.05  # y값 보정

        self.lidar_points = lidar_points

    def process_detections(self):
        """ YOLO 탐지 후 객체별 LiDAR 처리 및 퍼블리시 """
        res = self.model.detect(self.img_bgr)

        for det in res:
            bbox_x, bbox_y, bbox_width, bbox_height, conf, label = det
            label = int(label)

            # 바운딩 박스 그리기
            cv2.rectangle(self.img_bgr, (int(bbox_x), int(bbox_y)), 
                          (int(bbox_x + bbox_width), int(bbox_y + bbox_height)), (0, 255, 0), 2)
            cv2.putText(self.img_bgr, f"{label}: {conf:.2f}", 
                        (int(bbox_x), int(bbox_y) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # LiDAR 포인트 클라우드 시각화
        self.visualize_lidar()

    def visualize_lidar(self):
        """ LiDAR 포인트를 카메라 이미지에 오버레이 (간이 캘리브레이션 적용) """
        if self.lidar_points is None:
            return

        for point in self.lidar_points:
            x, y = int(point[0] * 10 + 320), int(480 - point[1] * 10)  # 픽셀 스케일 변환
            if 0 <= x < 640 and 0 <= y < 480:
                cv2.circle(self.img_bgr, (x, y), 2, (0, 0, 255), -1)

        cv2.imshow("YOLO & LiDAR", self.img_bgr)
        cv2.waitKey(1)

def main():
    rclpy.init()
    detector = ObstacleDetection()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
