#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Int16, Float32, UInt8, Bool
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from morai_msgs.msg import CtrlCmd,EventInfo, CollisionData
from morai_msgs.srv import MoraiEventCmdSrv
import numpy as np
import time
import matplotlib.pyplot as plt

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

MAXIMUM_I_TERM = 1.0

def i_term_winding(i_term, Ki, error, dt):
    if(abs(i_term) < MAXIMUM_I_TERM):
        i_term += Ki * error * dt
        return i_term
    else:
        if(i_term >= MAXIMUM_I_TERM):
            if (error <= 0):
                i_term += Ki * error * dt
                return i_term
            else:
                i_term = MAXIMUM_I_TERM
                return i_term
        elif(i_term <= -MAXIMUM_I_TERM):
            if(error >= 0):
                i_term += Ki * error * dt
                return i_term
            else:
                i_term = -MAXIMUM_I_TERM
                return i_term

class PI_Speed_controller():
    def __init__(self):
        self.Kp_speed = (4 * 0.45) * pow(0.9,5) # 4.0 -> 0.4s, Ziegler-Nichols 튜닝방법
        self.Tu = 0.35
        self.Ki_speed = (1.2 * self.Kp_speed / self.Tu) * pow(0.9,5) # temporary param value
        self.speed_error = 0
        self.i_term_speed = 0

    def calculate_error(self, desired_speed, current_speed):
        self.speed_error = desired_speed - current_speed
        print(f"desired speed: {desired_speed}")
        print(f"actual speed: {current_speed}")
    def command(self, desired_speed, current_speed, dt):
        #print(f" desired speed : {desired_speed}, current speed: {current_speed}")
        self.calculate_error(desired_speed, current_speed)
        self.i_term_speed = i_term_winding(self.i_term_speed, self.Ki_speed, self.speed_error, dt)
        return self.Kp_speed * self.speed_error + self.i_term_speed

    def reinit(self):
        self.speed_error = 0
        self.i_term_speed = 0

def emergency_stop():
    return -1
