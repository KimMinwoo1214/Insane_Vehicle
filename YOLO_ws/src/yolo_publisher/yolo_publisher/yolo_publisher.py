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
from sklearn.cluster import DBSCAN
from math import cos, sin, sqrt

# 하이퍼파라미터
LAVA_CONE_WIDTH_THRESHOLD = 50
LAVA_CONE_HEIGHT_THRESHOLD = 50
DRUM_WIDTH_THRESHOLD = 80
DRUM_HEIGHT_THRESHOLD = 80
TUNNEL_WIDTH_THRESHOLD = 100
TUNNEL_HEIGHT_THRESHOLD = 100

CAMERA_FOV = 70
LIDAR_FOV = 270
DBSCAN_EPS = 0.5
DBSCAN_MIN_SAMPLES = 5

class TrtYOLOv8:
    def __init__(self, engine_path):
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            runtime = trt.Runtime(self.logger)
            self.engine = runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()
        self.input_idx = self.engine.get_binding_index("images")
        self.output_idx = self.engine.get_binding_index("output0")

        self.input_shape = self.engine.get_binding_shape(self.input_idx)
        self.output_shape = self.engine.get_binding_shape(self.output_idx)

        self.input_size = np.prod(self.input_shape) * np.dtype(np.float32).itemsize
        self.output_size = np.prod(self.output_shape) * np.dtype(np.float32).itemsize

        self.d_input = cuda.mem_alloc(self.input_size)
        self.d_output = cuda.mem_alloc(self.output_size)
        self.stream = cuda.Stream()

    def preprocess(self, img):
        img = cv2.resize(img, (640, 640))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)
        return np.ascontiguousarray(img)

    def detect(self, img):
        img_input = self.preprocess(img)
        cuda.memcpy_htod_async(self.d_input, img_input, self.stream)
        self.context.execute_v2([self.d_input, self.d_output])
        output = np.empty(self.output_shape, dtype=np.float32)
        cuda.memcpy_dtoh_async(output, self.d_output, self.stream)
        self.stream.synchronize()
        return output.reshape(-1, 6)

class ObstacleDetection(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.object_info_pub = self.create_publisher(String, "/object_info", 10)
        self.tunnel_info_pub = self.create_publisher(String, "/tunnel_info", 10)

        self.create_subscription(Image, "/image_jpeg", self.image_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)

        self.model = TrtYOLOv8('/path/to/yolov8.engine')  # 경로 수정

        self.bridge = CvBridge()
        self.img_bgr = None
        self.lidar_points = None
        self.filtered_points = None

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if self.img_bgr is not None:
            self.process_detections()

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        points = np.array([ranges * np.cos(angles), ranges * np.sin(angles)]).T
        roi_mask = (points[:, 0] > 0.5) & (points[:, 0] < 10.0) & (abs(points[:, 1]) < 3.0)
        self.filtered_points = points[roi_mask]

    def process_detections(self):
        results = self.model.detect(self.img_bgr)

        tunnel_detected = False
        left_wall, right_wall = None, None

        for det in results:
            x, y, w, h, conf, label = det
            label = int(label)
            if label == 0 and w > LAVA_CONE_WIDTH_THRESHOLD and h > LAVA_CONE_HEIGHT_THRESHOLD:
                self.cluster_lidar_for_object(x, label)
            elif label == 1 and w > DRUM_WIDTH_THRESHOLD and h > DRUM_HEIGHT_THRESHOLD:
                self.cluster_lidar_for_object(x, label)
            elif label == 2 and w > TUNNEL_WIDTH_THRESHOLD and h > TUNNEL_HEIGHT_THRESHOLD:
                tunnel_detected = True
                left_wall, right_wall = self.estimate_tunnel_walls()

        self.tunnel_info_pub.publish(String(data=f"tunnel,{int(tunnel_detected)},{left_wall},{right_wall}"))

    def cluster_lidar_for_object(self, bbox_x, label):
        if self.filtered_points is None or len(self.filtered_points) == 0:
            return

        angle_ratio = bbox_x / 640
        lidar_angle = (angle_ratio * LIDAR_FOV) - (LIDAR_FOV / 2)
        angle_min = np.deg2rad(lidar_angle - 10)
        angle_max = np.deg2rad(lidar_angle + 10)

        angles = np.arctan2(self.filtered_points[:, 1], self.filtered_points[:, 0])
        mask = (angles > angle_min) & (angles < angle_max)
        selected_points = self.filtered_points[mask]

        if len(selected_points) == 0:
            return

        dbscan = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES)
        labels = dbscan.fit_predict(selected_points)

        for cluster_id in set(labels):
            if cluster_id == -1:
                continue
            cluster = selected_points[labels == cluster_id]
            cx = np.mean(cluster[:, 0])
            cy = np.mean(cluster[:, 1])
            dist = sqrt(cx**2 + cy**2)
            self.object_info_pub.publish(String(data=f"object,{label},{cx:.2f},{cy:.2f},{dist:.2f}"))

    def estimate_tunnel_walls(self):
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
