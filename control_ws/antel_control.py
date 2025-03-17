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

    if len(x) < 3:  # 최소 3개 이상의 포인트가 필요함
        return np.poly1d([0]), 0.0

    # 3차 다항식 피팅
    z = np.polyfit(x, y, 3)
    p = np.poly1d(z)

    # 미분
    first_derivative = p.deriv()
    second_derivative = first_derivative.deriv()

    curvature_list = [min(1e3, abs(second_derivative(x[i]) / ((1 + first_derivative(x[i]) ** 2) ** 1.5))) for i in range(len(x))]
    curvature = max(curvature_list) if len(curvature_list) > 0 else 0.0

    return first_derivative, curvature

class PurePursuit:
    """Pure Pursuit 기반 조향 제어"""
    def __init__(self):
        self.lfd = LFD_MAX
        self.ref_vel = 0
        self.theta = 0
        self.actual_look_ahead_point_x = 0

    def update_params(self, look_ahead_point):
        """Look-ahead Point 설정 및 조향각 계산"""
        self.actual_look_ahead_point_x = look_ahead_point.x + WHEELBASE  # 조향은 앞바퀴 기준이므로 WHEELBASE 차이 보정
        self.lfd = sqrt(pow(self.actual_look_ahead_point_x, 2) + pow(look_ahead_point.y, 2))
        self.theta = angle_clip(atan2(look_ahead_point.y, self.actual_look_ahead_point_x))

    def command(self, look_ahead_point):
        """조향 명령 생성"""
        self.update_params(look_ahead_point)
        print(f'Look-ahead point: {self.actual_look_ahead_point_x}, {look_ahead_point.y}')
        print(f'LFD: {self.lfd}')

        steering_command = angle_clip(atan2(2 * WHEELBASE * sin(self.theta), self.lfd))
        print(f'Lateral command: {steering_command}')
        return steering_mapping(steering_command)

class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

        self.create_subscription(UInt8, "behavior", self.behavior_callback, 10)
        self.create_subscription(NavSatFix, "gps_topic", self.gps_callback, 10)
        self.create_subscription(Path, "local_path", self.path_callback, 10)
        self.create_subscription(Image, "camera_topic", self.camera_callback, 10)

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
        """경로 데이터 업데이트"""
        self.local_path = msg
        _, self.curvature = calc_curvature_and_slope(self.local_path)
        self.is_path = True

    def control_loop(self):
        """제어 루프 실행"""
        self.lateral_control()
        self.publish()

    def lateral_control(self):
        """곡률 기반 Look-ahead Distance 조정 (Pure Pursuit)"""
        if self.is_path and len(self.local_path.poses) > 0:
            self.lfd = LFD_MIN + (LFD_MAX - LFD_MIN) * (1 - self.curvature / 1e3)
            self.lfd = max(LFD_MIN, min(self.lfd, LFD_MAX))

            self.look_ahead_point = self.local_path.poses[min(len(self.local_path.poses)-1, int(self.lfd))].pose.position
            self.steering_angle = self.pure_pursuit.command(self.look_ahead_point)

    def publish(self):
        """Arduino에 명령 전송"""
        command_str = f"{self.steering_angle},{MAXIMUM_SPEED}"
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
