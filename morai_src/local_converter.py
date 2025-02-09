#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from math import cos, sin
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from utils.functions import calc_curvature
import time
import json
import os

class LocalConverter():
    def __init__(self):
        rospy.init_node('local_converter', anonymous=True)
        self.global_path_sub = rospy.Subscriber('/waypoints', Path, self.path_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.curvature_pub = rospy.Publisher('/curvature', Float32, queue_size=1)

        self.global_path = Path()
        self.local_path = Path()
        self.curvature = Float32()
        self.current_position = Point()
        self.vehicle_yaw = 0
        self.is_global_path = False
        self.is_odom = False
        #self.is_parking = False
        self.is_first = True
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.is_global_path and self.is_odom :
                start_time = time.time()
                self.local_path = self.convert_global_path_to_local(self.global_path, self.current_position)
                #self.local_path.header.frame_id='odom'
                self.local_path.header.frame_id='base_link'
                self.curvature = calc_curvature(self.local_path)
                print("curvature:", self.curvature)
                self.curvature_pub.publish(self.curvature)
                self.local_path_pub.publish(self.local_path)
                self.is_global_path = self.is_odom = False
                print("elapsed time : ", time.time() - start_time)

            rate.sleep()

    def path_callback(self, msgs):
        self.global_path = msgs
        self.is_global_path = True

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

    def convert_global_path_to_local(self, path, vehicle_position):
        new_path = Path()
        #new_path.header.frame_id='odom'
        new_path.header.frame_id='base_link'
        translation = [vehicle_position.x, vehicle_position.y]

        t = np.array([
                    [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
                    [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
                    [0, 0, 1]])
        det_t = np.array([
                    [t[0][0], t[1][0], - (t[0][0] * translation[0] + t[1][0] * translation[1])],
                    [t[0][1], t[1][1], - (t[0][1] * translation[0] + t[1][1] * translation[1])],
                    [0, 0, 1]])

        for idx, point in enumerate(path.poses):
            x = point.pose.position.x
            y = point.pose.position.y
            point_matrix = np.array([[x], [y], [1]])
            local_point = det_t.dot(point_matrix)
            # print(local_point[0][0], local_point[1][0], idx)
            path.poses[idx].pose.position.x = local_point[0][0]
            path.poses[idx].pose.position.y = local_point[1][0]
            path.poses[idx].pose.position.z = 0
            path.poses[idx].pose.orientation.x = 0
            path.poses[idx].pose.orientation.y = 0
            path.poses[idx].pose.orientation.z = 0
            path.poses[idx].pose.orientation.w = 1
            new_path.poses.append(path.poses[idx])
            # print(path.poses[idx])

        return new_path

if __name__ == '__main__':
    try:
        local_converter = LocalConverter()
    except rospy.ROSInterruptException:
        pass