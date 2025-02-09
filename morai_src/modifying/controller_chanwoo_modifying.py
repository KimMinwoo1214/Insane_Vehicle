#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
from morai_msgs.msg import CtrlCmd,EventInfo, CollisionData, EgoVehicleStatus
from morai_msgs.srv import MoraiEventCmdSrv
import numpy as np
import time

from utils.lateral_controller import Pure_Pursuit, Stanley
from utils.longitudinal_controller import PI_Speed_controller, PI_Spacing_controller, emergency_stop
from utils.functions import calc_curvature

KMH_2_MS = 1/3.6
MAXIMUM_SPEED = 45*KMH_2_MS
MINIMUM_SPEED = 20*KMH_2_MS
MAX_WHEEL_ANGLE = (40*np.pi/180)

# ioniq 5 spec
VEHICLE_LENGTH = 4.635
VEHICLE_WIDTH = 1.892
VEHICLE_HEIGHT = 2.434
WHEELBASE = 3.0
FRONT_OVERHANG = 0.845
REAR_OVERHANG = 0.7
MINIMUM_TURNING_RADIUS = 5.97
MAXIMUM_CURVATURE = pow(MINIMUM_TURNING_RADIUS, -1)
# Behavior Macro
KEEPING_WAYPOINT = 0
EMERGENCY_STOP = 1
YIELD_TO_VEHICLE = 2 # ACC
LATTICE_PLAN = 3
TRAFFIC_STOP = 4

def angle_clip(x):
    return (x + np.pi) % (2 * np.pi) - np.pi

class Controller():
    def __init__(self):
        # print("hello world")
        rospy.init_node("controller")
        rospy.Subscriber("behavior", UInt8, self.behavior_callback)
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("lattice_path", Path, self.lattice_path_callback)
        rospy.Subscriber("vehicle_distance", Float32, self.distance_callback) # need to be fixed. Can you publish this topic Sumin Lee?
        rospy.Subscriber('CollisionData', CollisionData, self.collision_callback)
        rospy.Subscriber("Competition_topic", EgoVehicleStatus, self.status_callback)

        self.event_info_pub = rospy.Publisher('event_info', EventInfo, queue_size=1)
        self.ctrl_cmd_publisher = rospy.Publisher("ctrl_cmd", CtrlCmd, queue_size=10)

        self.Collision_check = CollisionData()
        self.collision_count = 0

        self.ctrl_cmd = CtrlCmd()
        self.ctrl_cmd.longlCmdType = 1

        self.behavior = 0
        self.local_path = Path()
        self.lattice_path = Path()
        self.look_ahead_point = Point()
        self.heading_angle_errors = None
        self.heading_angle_of_look_ahead_point = 0
        # for ACC controller (speed <-> spacing)
        self.speed_controller = PI_Speed_controller()
        self.spacing_controller = PI_Spacing_controller()
        self.longitudinal_command = 0
        self.curvature = 0
        self.curvature_lattice = 0
        self.current_speed = 0
        self.desired_speed = MAXIMUM_SPEED
        self.speed_curvature_term = 0
        self.speed_curvature_term_lattice = 0
        self.speed_steering_term = 0
        self.current_spacing = 0
        self.desired_spacing = 10 # temporary

        # for lateral control
        self.lateral_controller_pure_pursuit = Pure_Pursuit()
        self.lateral_controller_stanley = Stanley()
        self.lateral_command = 0
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

        if (self.behavior == KEEPING_WAYPOINT or self.behavior == LATTICE_PLAN): # speed control
            self.spacing_controller.reinit()
            self.longitudinal_command = self.speed_controller.command(self.desired_speed, self.current_speed, dt)

        elif (self.behavior == YIELD_TO_VEHICLE): # spacing control
            self.speed_controller.reinit()
            self.longitudinal_command = self.spacing_controller.command(self.desired_spacing, self.current_spacing, dt)

        elif (self.behavior == EMERGENCY_STOP or self.behavior == TRAFFIC_STOP): # AEB
            self.longitudinal_command = emergency_stop()

        if (self.longitudinal_command >= 0):
                self.ctrl_cmd.accel = self.longitudinal_command
                self.ctrl_cmd.brake = 0
        else:
            self.ctrl_cmd.accel = 0
            self.ctrl_cmd.brake = -self.longitudinal_command

        if self.Collision_check.collision_object is not None:
            self.collision_count += 1
            if self.collision_count >= 20:
                pass # gear change to rear and escape

    def lateral_control(self):
        # codes below are for stanley algorithm
        if self.is_path:
            if (self.behavior is not EMERGENCY_STOP and self.behavior is not TRAFFIC_STOP):
                next_point = Point()

                if self.behavior == LATTICE_PLAN:
                    self.look_ahead_point = self.lattice_path.poses[4].pose.position
                    next_point = self.lattice_path.poses[5].pose.position
                    self.heading_angle_of_look_ahead_point = angle_clip(atan2((next_point.y-self.look_ahead_point.y),(next_point.x-self.look_ahead_point.x)))
                    self.ctrl_cmd.steering = self.lateral_controller_stanley.command(self.current_speed, self.look_ahead_point, self.heading_angle_of_look_ahead_point)
                    self.speed_steering_term = (45 - 25.0 * min(MAX_WHEEL_ANGLE, abs(self.ctrl_cmd.steering))/MAX_WHEEL_ANGLE)*KMH_2_MS
                    self.desired_speed = self.alpha * self.speed_curvature_term + (1 - self.alpha) * self.speed_steering_term_lattice

                else:
                    self.look_ahead_point = self.local_path.poses[4].pose.position
                    next_point = self.local_path.poses[5].pose.position
                    self.heading_angle_of_look_ahead_point = angle_clip(atan2((next_point.y-self.look_ahead_point.y),(next_point.x-self.look_ahead_point.x)))
                    self.ctrl_cmd.steering = self.lateral_controller_stanley.command(self.current_speed, self.look_ahead_point, self.heading_angle_of_look_ahead_point)
                    self.speed_steering_term = (45 - 25.0 * min(MAX_WHEEL_ANGLE, abs(self.ctrl_cmd.steering))/MAX_WHEEL_ANGLE)*KMH_2_MS
                    self.desired_speed = self.alpha * self.speed_curvature_term + (1 - self.alpha) * self.speed_steering_term

        else: self.ctrl_cmd.steering = 0

    def publish(self):
        self.ctrl_cmd_publisher.publish(self.ctrl_cmd)

    def behavior_callback(self, msg):
        self.behavior = msg.data

    def path_callback(self, msg):
        if self.is_path == False:
            self.is_path = True
        self.local_path = msg
        self.curvature = calc_curvature(self.local_path)
        self.speed_curvature_term = self.desired_speed = (45-25.0*min(self.curvature, MAXIMUM_CURVATURE)/MAXIMUM_CURVATURE)*KMH_2_MS

    def collision_callback(self, msg):
        self.Collision_check = msg

    def status_callback(self, msg):
        self.current_speed = msg.velocity.x

    def distance_callback(self, msg):
        self.current_spacing = msg.data

    def lattice_path_callback(self, msg):
        self.lattice_path = msg
        self.curvature_lattice = calc_curvature(self.local_path)
        self.speed_curvature_term_lattice = self.desired_speed = (45-25.0*min(self.curvature_lattice, MAXIMUM_CURVATURE)/MAXIMUM_CURVATURE)*KMH_2_MS

    def serve_event_gear(self, gear):
        event_info_msg = EventInfo()
        event_info_msg.option = 2
        event_info_msg.ctrl_mode = 3
        event_info_msg.gear = gear
        self.event_cmd_service(event_info_msg)

    def finish(self):
        start_time = time.time()
        while time.time() - start_time < 1.5:
            self.ctrl_cmd.brake = 1.0
            self.ctrl_cmd.steering = 0
            self.ctrl_cmd_publisher.publish(self.ctrl_cmd)
        self.serve_event_gear(1)

if __name__ == "__main__":
    try:
        controller = Controller()

    except rospy.ROSInterruptException:
        pass