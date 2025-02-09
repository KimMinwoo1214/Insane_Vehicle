#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos, sin, pi, sqrt, pow, atan2
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete
import numpy as np
import time
import rospy
from geometry_msgs.msg import Point

STEERING_THRESHOLD = 0.007 # about 3 degree
LFD_MAX = 20

# ioniq 5 spec
MINIMUM_TURNING_RADIUS = 5.97
MAXIMUM_CURVATURE = pow(MINIMUM_TURNING_RADIUS, -1)
MAX_WHEEL_ANGLE = (40*np.pi/180)
VEHICLE_LENGTH = 4.635
VEHICLE_WIDTH = 1.892
VEHICLE_HEIGHT = 2.434
WHEELBASE = 3.0
FRONT_OVERHANG = 0.845
REAR_OVERHANG = 0.7
EPSILON = 1e-1

# vehicle parameters
m = 2050
Iz = 3600
lf = 3.0
lr = 0.0
Cf = 80000
Cr = 80000
I_psi = Iz

# Behavior Macro
KEEPING_WAYPOINT = 0
EMERGENCY_STOP = 1
YIELD_TO_VEHICLE = 2 # ACC
GPS_BLACKOUT = 3
LATTICE_PLAN = 4
LANE_DETECTION = 7
TRAFFIC_STOP = 8

def steering_mapping(x): #조향각의 물리적 한계 제한한
    return np.clip(x, -MAX_WHEEL_ANGLE, MAX_WHEEL_ANGLE)

def angle_clip(x):
    # 각도를 -pi에서 pi 사이로 클리핑
    return (x + np.pi) % (2 * np.pi) - np.pi

def smoothing_command(x): #미세한 각도제어(2도 이하) 무시 - 진동 제어어
    if abs(x) >= 2.0*np.pi/180:
        return x
    else: return 0

def rad2deg(rad_angle):
    return rad_angle * 180 / pi

def deg2rad(deg_angle):
    return deg_angle * pi / 180

class Pure_Pursuit():
    def __init__(self):
        self.lfd = LFD_MAX
        self.ref_vel = 0
        self.theta = 0
        self.actual_look_ahead_point_x = 0

    def update_params(self, look_ahead_point):
        self.actual_look_ahead_point_x = look_ahead_point.x + WHEELBASE #회전은 뒷바퀴 기준이고 조향은 앞바귀가 해서 휠베이스만큼 차이남
        self.lfd = sqrt(pow(self.actual_look_ahead_point_x,2) + pow(look_ahead_point.y, 2))
        self.theta = angle_clip(atan2(look_ahead_point.y, self.actual_look_ahead_point_x))

    def command(self, look_ahead_point):
        self.update_params(look_ahead_point)
        print(f'look ahead point; {self.actual_look_ahead_point_x},{look_ahead_point.y}')
        print(f'lfd; {self.lfd}')

        steering_command = angle_clip(atan2(2*WHEELBASE*sin(self.theta), self.lfd))
        print(f'lateral command; {steering_command}')
        return steering_mapping(steering_command)
