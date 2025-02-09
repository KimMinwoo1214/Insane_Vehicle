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
from morai_msgs.msg import CtrlCmd,EventInfo, CollisionData, EgoVehicleStatus, Lamps
from morai_msgs.srv import MoraiEventCmdSrv
import numpy as np
import time
from scipy.spatial import KDTree

from utils.lateral_controller import Pure_Pursuit, Stanley, pp_stanley_combined
from utils.longitudinal_controller import PI_Speed_controller, PI_Spacing_controller, emergency_stop
from utils.functions import calc_curvature_and_slope

KMH_2_MS = 1/3.6
MAXIMUM_SPEED = 30*KMH_2_MS
MINIMUM_SPEED = 20*KMH_2_MS
GRAVITY_CONSTANT = 9.81
MAX_WHEEL_ANGLE = (40*np.pi/180)
MU = 0.75 # friction coefficient

# ioniq 5 spec
VEHICLE_LENGTH = 4.635
VEHICLE_WIDTH = 1.892
VEHICLE_HEIGHT = 2.434
WHEELBASE = 3.0
FRONT_OVERHANG = 0.830
REAR_OVERHANG = 0.7
MINIMUM_TURNING_RADIUS = 5.97
MAXIMUM_CURVATURE = pow(MINIMUM_TURNING_RADIUS, -1)
# Behavior Macro
KEEPING_WAYPOINT = 0
EMERGENCY_STOP = 1
YIELD_TO_VEHICLE = 2 # ACC
LATTICE_PLAN = 3
TRAFFIC_STOP = 4
FINISH = 5

LFD_MAX = 20
LFD_MIN = 0.5

def angle_clip(x):
    # 각도를 -pi에서 pi 사이로 클리핑
    return (x + np.pi) % (2 * np.pi) - np.pi

class Controller():
    def __init__(self):
        # print("hello world")
        rospy.init_node("controller")
        rospy.Subscriber("behavior", UInt8, self.behavior_callback)
        rospy.Subscriber("lattice_path", Path, self.path_callback)
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        ############################################################
        # rospy.Subscriber("local_path", Path, self.path_callback) #
        ############################################################
        rospy.Subscriber("vehicle_distance", Float32, self.distance_callback)
        rospy.Subscriber('CollisionData', CollisionData, self.collision_callback)
        rospy.Subscriber("Competition_topic", EgoVehicleStatus, self.status_callback)

        self.event_info_pub = rospy.Publisher('event_info', EventInfo, queue_size=1)
        self.ctrl_cmd_publisher = rospy.Publisher("ctrl_cmd", CtrlCmd, queue_size=10)

        self.Collision_check = CollisionData()
        self.collision_count = 0

        self.ctrl_cmd = CtrlCmd()
        self.ctrl_cmd.longlCmdType = 1

        self.lamps = Lamps()
        self.lamps.turnSignal = 0

        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.event_cmd_service = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)

        self.behavior = 0
        self.local_path = Path()
        self.look_ahead_point = Point()
        self.look_ahead_point_stanley = Point()
        self.heading_angle_errors = None
        self.heading_angle_of_look_ahead_point = 0
        # for ACC controller (speed <-> spacing)
        self.speed_controller = PI_Speed_controller()
        self.spacing_controller = PI_Spacing_controller()
        self.longitudinal_command = 0
        self.curvature = 0
        self.current_speed = 0
        self.desired_speed = MAXIMUM_SPEED
        self.speed_curvature_term = 0
        self.speed_steering_term = 0
        self.current_spacing = 0
        self.desired_spacing = 15 # temporary
        self.current_position = [0, 0]

        # for lateral control
        self.lateral_controller_pure_pursuit = Pure_Pursuit()
        self.lateral_controller_stanley = Stanley()
        self.lateral_controller_combined = pp_stanley_combined()
        self.lateral_command = 0
        self.last_time = rospy.Time.now()

        self.alpha = 0.5
        self.is_path = False
        rate = rospy.Rate(40)

        while not rospy.is_shutdown():
            if len(self.Collision_check.collision_object) > 0:
                self.back_and_restart()
            self.lateral_control()
            self.longitudinal_control()
            self.publish()
            rate.sleep()

    # this was for pure pursuit
    def keeping_waypoint(self):
        self.lfd = 2.2*sqrt(max(self.current_speed,1e-1))
        for waypoint in self.local_path.poses:
            path_point = waypoint.pose.position
            dis = sqrt(pow(path_point.x, 2) + pow(path_point.y, 2))

            if self.lfd >= LFD_MAX:
                self.look_ahead_point = self.local_path.poses[39].pose.position #39번째가 max인 20m정도 거리인듯
                break
            elif dis >= self.lfd:
                self.look_ahead_point = path_point
                self.heading_angle_of_look_ahead_point = atan(self.heading_angle_errors(self.look_ahead_point.x))
                break

    def longitudinal_control(self):
        current_time = rospy.Time.now()
        dt = current_time.to_sec() - self.last_time.to_sec()
        self.last_time = current_time

        if (self.behavior == KEEPING_WAYPOINT or self.behavior == LATTICE_PLAN): # speed control
            self.spacing_controller.reinit()
            self.longitudinal_command = self.speed_controller.command(self.desired_speed, self.current_speed, dt)

        elif (self.behavior == YIELD_TO_VEHICLE): # spacing control
            self.speed_controller.reinit()
            start_time = time.time()
            while time.time() - start_time < 0.3:
                self.ctrl_cmd.brake = 1.0
                self.ctrl_cmd.steering = 0
                self.ctrl_cmd_publisher.publish(self.ctrl_cmd)

        elif (self.behavior == EMERGENCY_STOP or self.behavior == TRAFFIC_STOP): # AEB
            self.longitudinal_command = emergency_stop()

        elif self.behavior == FINISH:
            self.finish()

        if (self.longitudinal_command >= 0):
                self.ctrl_cmd.accel = self.longitudinal_command
                self.ctrl_cmd.brake = 0
        else:
            self.ctrl_cmd.accel = 0
            self.ctrl_cmd.brake = -self.longitudinal_command

    def lateral_control(self):
        # codes below are for stanley algorithm
        if self.is_path:
            if self.behavior == LATTICE_PLAN:
                if(len(self.local_path.poses) >= 17):
                    self.look_ahead_point_stanley = self.local_path.poses[15].pose.position
                    next_point = self.local_path.poses[16].pose.position
            else:
                if(len(self.local_path.poses) >= 5):
                    self.look_ahead_point_stanley = self.local_path.poses[4].pose.position
                    next_point = self.local_path.poses[5].pose.position
                    self.heading_angle_of_look_ahead_point = angle_clip(atan2((next_point.y-self.look_ahead_point_stanley.y),(next_point.x-self.look_ahead_point_stanley.x)))
                else:
                    point = Point()
                    point.x = 3.0
                    point.y = 0.0
                    point.z = 0.0
                    self.look_ahead_point_stanley = point
                    self.heading_angle_of_look_ahead_point = atan2(point.y, point.x)

            self.heading_angle_of_look_ahead_point = angle_clip(atan2((next_point.y-self.look_ahead_point_stanley.y),(next_point.x-self.look_ahead_point_stanley.x)))
            #self.keeping_waypoint()
            if (self.behavior is not EMERGENCY_STOP and self.behavior is not TRAFFIC_STOP and self.behavior is not FINISH):
                self.ctrl_cmd.steering = self.lateral_controller_stanley.command(self.curvature, self.current_speed, self.look_ahead_point_stanley, self.heading_angle_of_look_ahead_point)
                self.speed_steering_term = (30 - 15.0 * min(MAX_WHEEL_ANGLE, abs(self.ctrl_cmd.steering))/MAX_WHEEL_ANGLE)*KMH_2_MS
                self.desired_speed = self.alpha * self.speed_curvature_term + (1 - self.alpha) * self.speed_steering_term
            else: self.ctrl_cmd.steering = 0

            self.is_path = False

    def publish(self):
        self.ctrl_cmd_publisher.publish(self.ctrl_cmd)

    def behavior_callback(self, msg):
        self.behavior = msg.data

    def path_callback(self, msg):
        self.local_path = msg
        self.heading_angle_errors, self.curvature = calc_curvature_and_slope(self.local_path)
        self.speed_curvature_term = self.desired_speed = (30-15.0*min(self.curvature, MAXIMUM_CURVATURE)/MAXIMUM_CURVATURE)*KMH_2_MS
        self.is_path = True

    def collision_callback(self, msg):
        self.Collision_check = msg

    def status_callback(self, msg):
        self.current_speed = msg.velocity.x

    def distance_callback(self, msg):
        self.current_spacing = msg.data

    def odom_callback(self, msg):
        if (self.behavior is not None and self.behavior == FINISH):
            self.current_position[0] = msg.pose.pose.position.x
            self.current_position[1] = msg.pose.pose.position.y

    def serve_event_gear(self, gear):
        event_info_msg = EventInfo()
        event_info_msg.option = 2
        event_info_msg.ctrl_mode = 3
        event_info_msg.gear = gear
        self.event_cmd_service(event_info_msg)

    def back_and_restart(self):
        self.collision_count += 1
        if self.collision_count >= 20:
            start_time = time.time()
            while time.time() - start_time < 1.5:
                self.ctrl_cmd_accel = 0.0
                self.ctrl_cmd.brake = 1.0
                self.ctrl_cmd.steering = 0.0
                self.ctrl_cmd_publisher.publish(self.ctrl_cmd)
            self.serve_event_gear(2)
            while time.time() - start_time < 3.0:
                self.ctrl_cmd_accel = 3.0
                self.ctrl_cmd.brake = 0.0
                self.ctrl_cmd.steering = 0.0
                self.ctrl_cmd_publisher.publish(self.ctrl_cmd)
            while time.time() - start_time < 4.5:
                self.ctrl_cmd_accel = 0.0
                self.ctrl_cmd.brake = 1.0
                self.ctrl_cmd.steering = 0.0
                self.ctrl_cmd_publisher.publish(self.ctrl_cmd)
            self.serve_event_gear(4)
            self.collision_count = 0

    def finish(self):

        while not (self.current_position[0] > 679.5):
            print("nonstop")
            self.desired_speed = 10.0 * KMH_2_MS
            self.spacing_controller.reinit()
            self.speed_controller.reinit()
            self.longitudinal_command = self.speed_controller.command(self.desired_speed, self.current_speed, 0.025)
            self.ctrl_cmd_publisher.publish(self.ctrl_cmd)
            time.sleep(0.1)


        # if self.current_position[0] > 679.5 and self.current_position[1] < -136.5:
        rospy.logwarn("stop!!!!!!!!!")
        start_time = time.time()
        while time.time() - start_time < 1.5:
            self.ctrl_cmd.brake = 1.0
            self.ctrl_cmd.steering = 0
            self.ctrl_cmd_publisher.publish(self.ctrl_cmd)
        self.serve_event_gear(1)

        # else:
        #     print("nonstop")
        #     self.desired_speed = 10.0 * KMH_2_MS
        #     self.spacing_controller.reinit()
        #     self.speed_controller.reinit()
        #     self.longitudinal_command = self.speed_controller.command(self.desired_speed, self.current_speed, 0.025)

if __name__ == "__main__":
    try:
        controller = Controller()
    except rospy.ROSInterruptException:
        pass