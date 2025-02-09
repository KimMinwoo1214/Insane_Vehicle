#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
src_path = os.path.dirname(__file__)
package_dir_path = os.path.dirname(src_path)
functions_path = "/home/mobilion/catkin_ws/src/aMAP/include"
sys.path.append(package_dir_path)
sys.path.append(functions_path)

import rospy
import numpy as np
from math import cos, sin, sqrt
import os
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Proj
from tf.transformations import euler_from_quaternion
from utils.functions import calb_tangent

from tf import TransformBroadcaster

class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)

        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
        self.yaw_pub = rospy.Publisher('/yaw', Float32, queue_size=1)

        self.is_imu=False
        self.is_gps=False

        self.proj_UTM = Proj(proj='utm',zone=52, ellps='WGS84', preserve_units=False)

        self.odom_msg=Odometry()

        self.odom_msg.header.frame_id='/map'
        self.odom_msg.child_frame_id='/base_link'

        self.yaw = 0
        self.current_velocity = [0, 0]
        self.theta = 0
        self.last_yaw = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.prev_x = 0
        self.prev_y = 0
        self.global_imu_point = 0
        self.convert_matrix_from_local_to_global = None
        self.blackout_x = 0
        self.blackout_y = 0

        self.broadcaster = TransformBroadcaster()

        rate = rospy.Rate(40)

        while not rospy.is_shutdown():
            if self.is_imu and self.is_gps:
                self.convertLL2UTM()
                self.odom_msg.header.frame_id='/odom'
                self.odom_pub.publish(self.odom_msg)
                self.broadcast_transform()
                self.last_yaw = self.yaw
                self.is_gps = self.is_imu = False
                # print(f"odom_msg is now being published at '/odom' topic!\n")
                # print('-----------------[ odom_msg ]---------------------')
                # print(self.odom_msg.pose)
                # print(self.odom_msg.twist)
            else:
                pass
            rate.sleep()

    def imu_callback(self, data):
        if data.orientation.w == 0:
            self.odom_msg.pose.pose.orientation.x = 0.0
            self.odom_msg.pose.pose.orientation.y = 0.0
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0
        else:
            self.odom_msg.pose.pose.orientation.x = data.orientation.x
            self.odom_msg.pose.pose.orientation.y = data.orientation.y
            self.odom_msg.pose.pose.orientation.z = data.orientation.z
            self.odom_msg.pose.pose.orientation.w = data.orientation.w

            odom_orientation = (data.orientation.x, data.orientation.y, data.orientation.z,
                           data.orientation.w)
            odom_euler = euler_from_quaternion(odom_orientation)

            self.yaw = odom_euler[2]
            print(f"yaw: {self.yaw}")

            linear_acceleration = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])
            linear_acceleration = calb_tangent(odom_euler, linear_acceleration)

            self.odom_msg.twist.twist.linear.x = linear_acceleration[0]
            self.odom_msg.twist.twist.linear.y = linear_acceleration[1]
            self.odom_msg.twist.twist.linear.z = linear_acceleration[2]


        self.is_imu = True

    def navsat_callback(self, gps_msg):
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset
        self.z = gps_msg.altitude
        if self.lat == 0 and self.lon == 0:
            self.is_gps = False
            self.is_enter_blackout = True
        else:
            self.is_gps=True

    def convertLL2UTM(self):
        xy_zone = self.proj_UTM(self.lon, self.lat)
        if self.lon == 0 and self.lat == 0:
            self.x = 0.0
            self.y = 0.0
        else:
            self.x = xy_zone[0] - self.e_o
            self.y = xy_zone[1] - self.n_o

        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = self.z

    def publish_yaw(self):
        self.yaw_pub.publish(Float32(self.yaw))

    def broadcast_transform(self):
        # Broadcasting the transformation from /odom to /base_link
        self.broadcaster.sendTransform(
            (self.x, self.y, self.z),  # Translation
            (self.odom_msg.pose.pose.orientation.x,
             self.odom_msg.pose.pose.orientation.y,
             self.odom_msg.pose.pose.orientation.z,
             self.odom_msg.pose.pose.orientation.w),  # Orientation as quaternion
            rospy.Time.now(),
            "/gps",  # Child frame
            "/map"        # Parent frame
        )
        self.broadcaster.sendTransform(
            (-3.00,0.01,-0.47),
            (0,0,0,1),
            rospy.Time.now(),
            "/base_link",
            "/gps"
        )
        self.broadcaster.sendTransform(
            (1.31, 0, 1.6),
            (0,0,0,1),
            rospy.Time.now(),
            "/velodyne",
            "/base_link"
        )
        self.broadcaster.sendTransform(
            (3.00, 0, 0.80),
            (0, 0, 0, 1),
            rospy.Time.now(),
            "/camera",
            "/base_link"
        )

if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass