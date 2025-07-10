#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rclpy
from rclpy.node import Node
import math
import numpy as np
import serial
import collections
import cv2

from math import sqrt, pow, sin, atan2
from sensor_msgs.msg import NavSatFix, Image
from nav_msgs.msg import Path
from std_msgs.msg import String, UInt8
from cv_bridge import CvBridge

# 차량 속도 및 제어 설정
MAXIMUM_SPEED = 30
MAX_WHEEL_ANGLE = (22.5 * np.pi / 180)
CENTER_STEERING = 90 
LEFT_MAX = 123       
RIGHT_MAX = 54     

KEEPING_WAYPOINT = 0
EMERGENCY_STOP = 1
LFD_MAX = 10  # 최대 Look-ahead Distance
LFD_MIN = 0.5  # 최소 Look-ahead Distance
WHEELBASE = 0.3  # 차량 앞바퀴와 뒷바퀴 간 거리

# GPS 기반 헤딩 보정 설정
GPS_UPDATE_INTERVAL = 0.1  # 초 단위
N = 5  # Moving Average 윈도우 크기
heading_history = collections.deque(maxlen=N)

# Optical Flow 설정
prev_gray = None
prev_pts = None

def angle_clip(angle):
    """조향각을 -π ~ π 범위로 제한"""
    return math.atan2(math.sin(angle), math.cos(angle))

def steering_mapping(steering_angle):
    """조향각을 PWM 신호로 변환"""
    return CENTER_STEERING + int(steering_angle / MAX_WHEEL_ANGLE * (LEFT_MAX - CENTER_STEERING))

def calc_curvature_and_slope(local_path_points):
    """경로의 곡률(curvature) 및 기울기(1차 미분) 계산"""
    x = np.array([point.pose.position.x for point in local_path_points.poses])
    y = np.array([point.pose.position.y for point in local_path_points.poses])

    if len(x) < 3:
        return np.poly1d([0]), 0.0

    z = np.polyfit(x, y, 3)
    p = np.poly1d(z)

    first_derivative = p.deriv()
    second_derivative = first_derivative.deriv()

    curvature_list = [min(1e3, abs(second_derivative(x[i]) / ((1 + first_derivative(x[i]) ** 2) ** 1.5))) for i in range(len(x))]
    curvature = max(curvature_list) if curvature_list else 0.0

    return first_derivative, curvature

class PurePursuit:
    """Pure Pursuit 기반 조향 제어"""
    def __init__(self):
        self.lfd = LFD_MAX
        self.ref_vel = 0
        self.theta = 0
        self.actual_look_ahead_point_x = 0

    def update_params(self, look_ahead_point):
        self.actual_look_ahead_point_x = look_ahead_point.x + WHEELBASE
        self.lfd = sqrt(pow(self.actual_look_ahead_point_x, 2) + pow(look_ahead_point.y, 2))
        self.theta = angle_clip(atan2(look_ahead_point.y, self.actual_look_ahead_point_x))

    def calculate_angle(self, look_ahead_point):
        self.update_params(look_ahead_point)
        print(f'Look-ahead point: {self.actual_look_ahead_point_x}, {look_ahead_point.y}')
        print(f'LFD: {self.lfd}')
        return angle_clip(atan2(2 * WHEELBASE * sin(self.theta), self.lfd))

class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

        #########근데 이거 플래닝에서 받아오는 거도 /붙여줘야하는 거 아닌가 모르겠음
        self.create_subscription(UInt8, "behavior", self.behavior_callback, 10)
        self.create_subscription(NavSatFix, "/fix", self.gps_callback, 10)
        self.create_subscription(Path, "local_path", self.path_callback, 10)
        self.create_subscription(Image, "/image_jpeg", self.camera_callback, 10)

        self.cmd_pub = self.create_publisher(String, "teleop_commands", 10)

        self.bridge = CvBridge()
        self.behavior = 0
        self.current_position = [0.0, 0.0]
        self.heading = 0.0
        self.last_gps_time = None

        self.local_path = Path()
        self.look_ahead_point = None
        self.pure_pursuit = PurePursuit()
        self.is_path = False

        self.timer = self.create_timer(1.0 / 40, self.control_loop)

    def gps_callback(self, msg):
        """GPS 데이터를 사용하여 차량 위치 및 헤딩 업데이트"""
        global heading_history
        x_prev, y_prev = self.current_position
        self.current_position = [msg.latitude, msg.longitude]

        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = (current_time - self.last_gps_time) if self.last_gps_time else GPS_UPDATE_INTERVAL
        self.last_gps_time = current_time

        dx = self.current_position[0] - x_prev
        dy = self.current_position[1] - y_prev
        speed = math.sqrt(dx**2 + dy**2) / dt

        if speed > 0.1:
            new_heading = math.atan2(dy, dx)
            heading_history.append(new_heading)

        if len(heading_history) > 1:
            self.heading = sum(heading_history) / len(heading_history)

    def path_callback(self, msg):
        """
        경로 데이터 업데이트 (local path일 때 기준으로 작성함. global path이면 주석처리한 부분 사용하기)
        """
        self.local_path = msg
        _, self.curvature = calc_curvature_and_slope(self.local_path)
        self.is_path = True

        # ---- global path를 받는 경우 (선택적으로 사용) ----
        # import utm
        # converted_path = []
        # for pose in msg.poses:
        #     lat = pose.pose.position.x
        #     lon = pose.pose.position.y
        #     utm_x, utm_y, _, _ = utm.from_latlon(lat, lon)
        #     converted_path.append((utm_x, utm_y))
        # self.local_path = convert_global_to_local(converted_path, self.current_position, self.heading)

    def control_loop(self):
        """제어 루프 실행"""
        self.lateral_control()
        self.publish()

    def lateral_control(self):
        """곡률 기반 Look-ahead Distance 조정 (Pure Pursuit)"""
        if self.is_path and len(self.local_path.poses) > 0:
            self.pure_pursuit.lfd = LFD_MIN + (LFD_MAX - LFD_MIN) * (1 - self.curvature / 1e3)
            self.pure_pursuit.lfd = max(LFD_MIN, min(self.pure_pursuit.lfd, LFD_MAX))

            idx = min(len(self.local_path.poses) - 1, round(self.pure_pursuit.lfd))
            self.look_ahead_point = self.local_path.poses[idx].pose.position
            self.steering_angle = self.pure_pursuit.calculate_angle(self.look_ahead_point)

    def camera_callback(self, msg):
        """Optical Flow를 이용한 헤딩 보정"""
        global prev_gray, prev_pts
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if prev_gray is None:
            prev_gray = gray
            prev_pts = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=0.3, minDistance=7)
            return

        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, None)

        if next_pts is not None and prev_pts is not None and len(next_pts) > 5:
            movement = next_pts - prev_pts
            avg_angle = np.mean(np.arctan2(movement[:, 0, 1], movement[:, 0, 0]))
            heading_change = np.rad2deg(avg_angle)

            prev_gray = gray
            prev_pts = next_pts

            gps_reliability = min(1.0, len(heading_history) / N)
            alpha = 0.7 * gps_reliability + 0.3

            self.heading = alpha * self.heading + (1 - alpha) * heading_change

    def publish(self):
        """Arduino에 명령 전송"""
        pwm = steering_mapping(self.steering_angle)
        command_str = f"{pwm},{MAXIMUM_SPEED}"
        self.cmd_pub.publish(String(data=command_str))
        self.ser.write((command_str + "\n").encode())

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
