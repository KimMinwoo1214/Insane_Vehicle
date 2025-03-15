#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rclpy
from rclpy.node import Node
from math import sqrt, pow, pi, atan2
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from std_msgs.msg import String, UInt8
import numpy as np
import serial
#######위치정보 x,y로 받았다고 가정하고 작성함. RDDF랑 fix정보를 x,y로 변환하는 노드 필요함########

from utils.lateral_controller import Pure_Pursuit
from utils.functions import calc_curvature_and_slope

# 속도 및 차량 스펙 설정
#PWM 스티어링 : 123(좌)~90(중앙)~54(우)
#PWM 속도 : 0~30
MAXIMUM_SPEED = 30  # 최대 속도 PWM
MAX_WHEEL_ANGLE = (22.5 * np.pi / 180)

MINIMUM_TURNING_RADIUS = 2.044
MAXIMUM_CURVATURE = pow(MINIMUM_TURNING_RADIUS, -1)
CENTER_STEERING = 90 
LEFT_MAX = 123       
RIGHT_MAX = 54     

KEEPING_WAYPOINT = 0
EMERGENCY_STOP = 1
LFD_MAX = 10
LFD_MIN = 0.5

   

class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # 아두이노 시리얼 통신 설정

        # ROS2 Subscription 설정
        self.create_subscription(UInt8, "behavior", self.behavior_callback, 10)
        self.create_subscription(NavSatFix, "gps_topic", self.gps_callback, 10)
        self.create_subscription(Path, "local_path", self.path_callback, 10)

        # ROS2 Publisher 설정 (teleop_commands)
        self.cmd_pub = self.create_publisher(String, "teleop_commands", 10)

        self.behavior = 0
        self.current_position = [0.0, 0.0]
        self.curvature = 0

        self.local_path = Path()
        self.look_ahead_point = None

        self.lateral_controller_pure_pursuit = Pure_Pursuit()
        self.is_path = False

        self.timer = self.create_timer(1.0 / 40, self.control_loop)  # 40Hz 실행

    def control_loop(self):
        """제어 루프 실행"""
        self.lateral_control()
        self.publish()

    def lateral_control(self):
        """횡방향 제어 (Pure Pursuit 기반)"""
        if self.is_path and len(self.local_path.poses) > 0:
            # 곡률 기반 Look-ahead Distance 계산
            curvature_factor = max(1.0 - self.curvature / MAXIMUM_CURVATURE, 0.5)
            self.lfd = max(LFD_MIN, min(LFD_MAX * curvature_factor, LFD_MAX))

            # Look-ahead Point 선택
            self.look_ahead_point = self.local_path.poses[-1].pose.position
            for waypoint in self.local_path.poses:
                path_point = waypoint.pose.position
                dis = sqrt(pow(path_point.x - self.current_position[0], 2) + pow(path_point.y - self.current_position[1], 2))
                if dis >= self.lfd:
                    self.look_ahead_point = path_point
                    break

            if self.behavior != EMERGENCY_STOP:
                self.steering_angle = self.lateral_controller_pure_pursuit.command(self.look_ahead_point)
            else:
                self.steering_angle = 0

            self.is_path = False

    def publish(self):
        """Arduino에 명령 전송 (steering, throttle)"""
        # 속도 설정: 최대 속도로 유지, 비상 정지 시 0
        throttle = 0 if self.behavior == EMERGENCY_STOP else MAXIMUM_SPEED

        # 조향 변환 (rad → PWM 스티어링)
        steering = CENTER_STEERING + int(self.steering_angle / MAX_WHEEL_ANGLE * (LEFT_MAX - CENTER_STEERING))
        steering = max(RIGHT_MAX, min(LEFT_MAX, steering))

        # "steering,throttle" 포맷으로 전송
        command_str = f"{steering},{throttle}"
        self.get_logger().info(f"전송: {command_str}")

        # ROS2 String 메시지로 퍼블리싱 (teleop_commands)
        msg = String()
        msg.data = command_str
        self.cmd_pub.publish(msg)

        # Arduino로 시리얼 전송
        try:
            self.ser.write((command_str + "\n").encode())
        except serial.SerialException:
            self.get_logger().warn("아두이노 전송 실패. 연결 확인 필요.")

    def behavior_callback(self, msg):
        """행동 모드 변경"""
        self.behavior = msg.data

    def path_callback(self, msg):
        """경로 데이터 업데이트"""
        self.local_path = msg
        _, self.curvature = calc_curvature_and_slope(self.local_path)
        self.is_path = True

    def gps_callback(self, msg):
        """GPS 데이터를 사용하여 차량 위치 업데이트"""
        self.current_position[0] = msg.latitude
        self.current_position[1] = msg.longitude

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

