#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 모라이sim 대회 베이스입니다.

import os, sys
src_path = os.path.dirname(__file__)
package_dir_path = os.path.dirname(src_path)
functions_path = os.path.join(package_dir_path, "include")
sys.path.append(package_dir_path)
sys.path.append(functions_path)

import rospy
from math import cos, sin, pi, sqrt, pow, atan2, atan
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Int16, Float32, UInt8, Bool
from morai_msgs.msg import CtrlCmd,EventInfo, EgoVehicleStatus
from morai_msgs.srv import MoraiEventCmdSrv
import numpy as np
import time
from scipy.spatial import KDTree
import serial


from utils.lateral_controller import Pure_Pursuit
from utils.longitudinal_controller import PI_Speed_controller, emergency_stop
from utils.functions import calc_curvature_and_slope

KMH_2_MS = 1/3.6
MAXIMUM_SPEED = 7*KMH_2_MS #바꾸기
MINIMUM_SPEED = 20*KMH_2_MS #바꾸기
GRAVITY_CONSTANT = 9.81
MAX_WHEEL_ANGLE = (40*np.pi/180) #최대각도 찾아서 바꾸기

# 차 스펙에 맞게 바꾸기
MINIMUM_TURNING_RADIUS = 5.97
MAXIMUM_CURVATURE = pow(MINIMUM_TURNING_RADIUS, -1)

KEEPING_WAYPOINT = 0
EMERGENCY_STOP = 1
LFD_MAX = 10
LFD_MIN = 0.5

def angle_clip(x):
    # 각도를 -pi에서 pi 사이로 클리핑
    return (x + np.pi) % (2 * np.pi) - np.pi

class Controller():
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 2000000, timeout=1)
        rospy.init_node("controller")
        rospy.Subscriber("behavior", UInt8, self.behavior_callback)
        rospy.Subscriber('odom', Odometry, self.odom_callback) #위치 데이터 구독 - 인지랑 이름 맞추기
        rospy.Subscriber("local_path", Path, self.path_callback) #경로 구독 - 플래닝과 이름 맞추기
        rospy.Subscriber("Competition_topic", EgoVehicleStatus, self.status_callback) #속도 데이터 구독 - 인지랑 이름 맞추기

        self.ctrl_cmd_publisher = rospy.Publisher("ctrl_cmd", CtrlCmd, queue_size=10)

        self.ctrl_cmd = CtrlCmd()
        self.ctrl_cmd.longlCmdType = 1

        self.behavior = 0
        self.current_speed = 0
        self.current_position = [0.0, 0.0]
        self.desired_speed = 0
        self.heading_angle_errors = lambda x: 0 #함수로 활용될 때 대비
        self.curvature = 0
        self.longitudinal_command = 0

        self.local_path = Path()
        self.look_ahead_point = Point()
        self.heading_angle_errors = None
        self.heading_angle_of_look_ahead_point = 0

        self.speed_controller = PI_Speed_controller()
        self.lateral_controller_pure_pursuit = Pure_Pursuit()
        self.last_time = rospy.Time.now()

        self.alpha = 0.5
        self.is_path = False
        rate = rospy.Rate(40)

        while not rospy.is_shutdown():
            self.lateral_control()
            self.longitudinal_control()
            self.publish()
            rate.sleep()

    def longitudinal_control(self):
        current_time = rospy.Time.now()
        dt = current_time.to_sec() - self.last_time.to_sec()
        self.last_time = current_time

        if (self.behavior == KEEPING_WAYPOINT): # speed control
            self.longitudinal_command = self.speed_controller.command(self.desired_speed, self.current_speed, dt)

        elif (self.behavior == EMERGENCY_STOP): # AEB
            self.longitudinal_command = emergency_stop()

        if (self.longitudinal_command >= 0):
                self.ctrl_cmd.accel = self.longitudinal_command
                self.ctrl_cmd.brake = 0
        else:
            self.ctrl_cmd.accel = 0
            self.ctrl_cmd.brake = -self.longitudinal_command
            
    def lateral_control(self):
        if self.is_path and len(self.local_path.poses) > 0:
            # Look-ahead Distance 계산 (속도 + 곡률 기반 조정)
            base_lfd = 2.2 * sqrt(max(self.current_speed, 1e-1))
            curvature_factor = max(1.0 - self.curvature / MAXIMUM_CURVATURE, 0.5)  # 급커브일수록 감소
            self.lfd = max(LFD_MIN, min(base_lfd * curvature_factor, LFD_MAX))

            # LFD 기반으로 Look-ahead Point 선택 (경로가 짧으면 마지막 점 사용)
            self.look_ahead_point = self.local_path.poses[-1].pose.position  # 기본값 설정
            for waypoint in self.local_path.poses:
                path_point = waypoint.pose.position
                dis = sqrt(pow(path_point.x, 2) + pow(path_point.y, 2))
                if dis >= self.lfd:
                    self.look_ahead_point = path_point
                    break

            # 스티어링링 계산 및 속도 조정
            if self.behavior != EMERGENCY_STOP:
                self.ctrl_cmd.steering = self.lateral_controller_pure_pursuit.command(self.look_ahead_point)
                
                # 급회전 시 속도 제한
                steering_factor = max(1.0 - abs(self.ctrl_cmd.steering) / MAX_WHEEL_ANGLE, 0.5)
                self.desired_speed = self.alpha * self.speed_curvature_term + (1 - self.alpha) * self.speed_steering_term * steering_factor

            self.is_path = False

    def publish(self):
        #PWM 변환 후 시리얼로 아두이노에 전송
        #속도 변환 (m/s → PWM 0~255)
        PWM_speed = int((min(self.desired_speed, MAXIMUM_SPEED) / MAXIMUM_SPEED) * 255)
        PWM_speed = max(0, min(PWM_speed, 255))  # 범위 제한

        #조향각 변환 (rad → PWM 1000~2000us)
        PWM_steering = int(1500 + (angle_clip(self.ctrl_cmd.steering) / MAX_WHEEL_ANGLE) * 500)
        PWM_steering = max(1000, min(PWM_steering, 2000))  # 범위 제한

        #시리얼 데이터 전송 ("조향PWM,속도PWM")
        command = f"{PWM_steering},{PWM_speed}\n"
        try:
            self.ser.write(command.encode())
        except serial.SerialException:
            rospy.logwarn("Failed to send serial data. Check Arduino connection.")

        # 혹시 모르니 기존 ROS 메시지도 유지
        self.ctrl_cmd_publisher.publish(self.ctrl_cmd)


    def behavior_callback(self, msg):
        self.behavior = msg.data

    def path_callback(self, msg):
        self.local_path = msg
        self.heading_angle_errors, self.curvature = calc_curvature_and_slope(self.local_path)
        self.speed_curvature_term = self.desired_speed = (30-15.0*min(self.curvature, MAXIMUM_CURVATURE)/MAXIMUM_CURVATURE)*KMH_2_MS #식 점검해보기기
        self.is_path = True

    def status_callback(self, msg):
        self.current_speed = msg.velocity.x

    def odom_callback(self, msg):
            self.current_position[0] = msg.pose.pose.position.x
            self.current_position[1] = msg.pose.pose.position.y

if __name__ == "__main__":
    try:
        controller = Controller()
    except rospy.ROSInterruptException:
        pass