#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from std_msgs.msg import Float32, UInt8
from morai_msgs.msg import CtrlCmd, CollisionData, EgoVehicleStatus
import numpy as np
import cvxpy as cp  # MPC 최적화 라이브러리

# 상수 및 차량 파라미터 정의
KMH_2_MS = 1 / 3.6
MAXIMUM_SPEED = 45 * KMH_2_MS
MINIMUM_SPEED = 20 * KMH_2_MS
MAX_WHEEL_ANGLE = (40 * np.pi / 180)
WHEELBASE = 3.0
MAXIMUM_CURVATURE = pow(5.97, -1)  # 최소 회전 반경 기반 최대 곡률

# 동작 매크로
KEEPING_WAYPOINT = 0
EMERGENCY_STOP = 1
YIELD_TO_VEHICLE = 2  # ACC
TRAFFIC_STOP = 4

def angle_clip(x):
    """각도를 -pi에서 pi 사이로 클리핑"""
    return (x + np.pi) % (2 * np.pi) - np.pi

class Controller():
    def __init__(self):
        rospy.init_node("controller")
        rospy.Subscriber("behavior", UInt8, self.behavior_callback)
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("vehicle_distance", Float32, self.distance_callback)
        rospy.Subscriber('CollisionData', CollisionData, self.collision_callback)
        rospy.Subscriber("Competition_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber('/curvature', Float32, self.curvature_callback)  # curvature 값 구독

        self.ctrl_cmd_publisher = rospy.Publisher("ctrl_cmd", CtrlCmd, queue_size=10)

        self.Collision_check = CollisionData()
        self.collision_count = 0

        self.ctrl_cmd = CtrlCmd()
        self.ctrl_cmd.longlCmdType = 1

        self.behavior = 0
        self.local_path = Path()
        self.look_ahead_point = Point()
        self.current_speed = 0
        self.desired_speed = MAXIMUM_SPEED
        self.current_spacing = 0
        self.desired_spacing = 10  # 임시 값
        self.curvature = 0  # 초기 curvature 값 설정
        self.prev_error_y = 0
        self.integral_error_y = 0

        # MPC 설정
        self.mpc_horizon = 30  # 예측 지평선
        self.dt = 0.1  # 샘플링 시간
        self.x = np.zeros((4, self.mpc_horizon + 1))  # 상태 변수 [x, y, yaw, v]
        self.u = np.zeros((2, self.mpc_horizon))  # 제어 변수 [steering, acceleration]
        self.Q = np.diag([100.0, 100.0, 6.0, 0.1])  # 상태 가중치
        self.R = np.diag([0.2, 0.25])  # 제어 가중치

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
        """종방향 제어: 속도 및 간격 제어"""
        current_time = rospy.Time.now()
        self.dt = current_time.to_sec() - self.last_time.to_sec()
        self.last_time = current_time

        if self.behavior == KEEPING_WAYPOINT:  # 기본 속도 유지
            self.longitudinal_command = self.desired_speed - self.current_speed

        elif self.behavior == YIELD_TO_VEHICLE:  # 간격 유지 제어
            self.longitudinal_command = self.desired_spacing - self.current_spacing

        elif self.behavior == EMERGENCY_STOP or self.behavior == TRAFFIC_STOP:  # 긴급 정지
            self.longitudinal_command = -self.current_speed * 10  # 강제 감속

        # 제어 명령 적용
        if self.longitudinal_command >= 0:
            self.ctrl_cmd.accel = min(self.longitudinal_command, 1.0)
            self.ctrl_cmd.brake = 0
        else:
            self.ctrl_cmd.accel = 0
            self.ctrl_cmd.brake = min(-self.longitudinal_command, 1.0)

        # 충돌 체크
        if self.Collision_check.collision_object is not None:
            self.collision_count += 1
            if self.collision_count >= 20:
                pass  # 후진으로 기어 변경 후 탈출

    def lateral_control(self):
        """측방향 제어: MPC와 PID 제어를 사용하여 조향각 계산"""
        if self.is_path:
            # 상태 변수 업데이트
            self.update_states()

            # MPC 문제 설정
            x = cp.Variable((4, self.mpc_horizon + 1))
            u = cp.Variable((2, self.mpc_horizon))

            cost = 0
            constraints = []

            # 예측 지평선 동안의 최적화 설정
            for t in range(self.mpc_horizon):
                cost += cp.quad_form(x[:, t] - self.x[:, t], self.Q) + cp.quad_form(u[:, t], self.R)
                # 선형 상태 갱신 모델
                constraints += [x[:, t + 1] == x[:, t] + self.dt * cp.hstack([x[3, t], 
                                                                              x[3, t],  
                                                                              u[0, t] / WHEELBASE, 
                                                                              u[1, t]])]
                constraints += [cp.abs(u[0, t]) <= MAX_WHEEL_ANGLE]  # 최대 조향각 제약
                constraints += [cp.abs(u[1, t]) <= 1.0]  # 최대 가속도 제약

            # 최적화 문제 풀기
            prob = cp.Problem(cp.Minimize(cost), constraints)
            prob.solve()

            # MPC 제어에서 계산된 조향각 적용
            self.ctrl_cmd.steering = -u.value[0, 0]

            # P, I, D 제어 추가 (y 축 경로 오차 보정)
            # error_y = self.local_path.poses[5].pose.position.y - self.x[1, 0]
            # k_p = 0.2  # 비례 상수
            # k_i = 0.02  # 적분 상수 (오차 누적에 따른 보정)
            # # 오차 변화율(D) 계산
            # delta_error_y = error_y - self.prev_error_y
            # # 적분 오차(I) 누적
            # self.integral_error_y += error_y * self.dt

            # # PID 제어 결합: MPC 조향각에 PID 보정 추가
            # self.ctrl_cmd.steering -= (k_p * error_y) + (k_i * self.integral_error_y)
        
            # # 이전 오차 업데이트
            # self.prev_error_y = error_y

            # 최적 가속도 및 제동 명령 적용
            self.ctrl_cmd.accel = max(0, u.value[1, 0])
            self.ctrl_cmd.brake = max(0, -u.value[1, 0])

    def update_states(self):
        """차량의 현재 상태를 업데이트"""
        self.x[0, 0] = self.look_ahead_point.x  # x 위치
        self.x[1, 0] = self.look_ahead_point.y  # y 위치
        self.x[2, 0] = self.heading_angle_of_look_ahead_point  # 차량의 헤딩
        self.x[3, 0] = self.current_speed  # 속도

    def publish(self):
        """제어 명령을 퍼블리시"""
        self.ctrl_cmd_publisher.publish(self.ctrl_cmd)

    def behavior_callback(self, msg):
        self.behavior = msg.data

    def path_callback(self, msg):
        if not self.is_path:
            self.is_path = True
        self.local_path = msg
        # 경로 정보가 업데이트되면, 차량의 상태 및 목표 속도를 조정합니다.
        self.look_ahead_point = self.local_path.poses[4].pose.position
        self.heading_angle_of_look_ahead_point = atan2(
            self.local_path.poses[5].pose.position.y - self.look_ahead_point.y,
            self.local_path.poses[5].pose.position.x - self.look_ahead_point.x,
        )
        self.speed_curvature_term = self.desired_speed = (45 - 25.0 * min(self.curvature, MAXIMUM_CURVATURE) / MAXIMUM_CURVATURE) * KMH_2_MS

    def curvature_callback(self, msg):
        """curvature 값을 업데이트"""
        self.curvature = msg.data

    def collision_callback(self, msg):
        self.Collision_check = msg

    def status_callback(self, msg):
        self.current_speed = msg.velocity.x

    def distance_callback(self, msg):
        self.current_spacing = msg.data

if __name__ == "__main__":
    try:
        controller = Controller()
    except rospy.ROSInterruptException:
        pass
