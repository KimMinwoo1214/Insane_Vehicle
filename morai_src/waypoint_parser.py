#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import UInt8, Float32, Bool
from geometry_msgs.msg import PoseStamped
import json
import os
from utils.functions import get_closest_index_kdtree
import time

LIMIT_DISTANCE = 10.0

class WaypointParser():
    def __init__(self):
        rospy.init_node('waypoint_parser', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.waypoints_pub = rospy.Publisher('/waypoints', Path, queue_size=5)
        self.linkindex_pub = rospy.Publisher('/link_index', UInt8, queue_size=1)
        self.max_speed_pub = rospy.Publisher('max_speed', Float32, queue_size=1)  # max_speed 퍼블리셔 추가

        self.ego_loc = []
        self.is_odom = False
        self.is_parking_done = False
        self.current_link_index = 0
        self.waypoints = Path()
        self.link_index = UInt8()
        self.global_waypoints = []

        path = "/home/mobilion/catkin_ws/src/aMAP/src/link/mando_path_origin_value_index.json"
        with open(path, 'r') as file:
            self.link_data = json.load(file)
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.is_odom:
                #start_time = time.time()
                self.global_waypoints = []
                self.create_and_publish_global_waypoints()
                self.is_odom = False
            rate.sleep()

    def create_and_publish_global_waypoints(self):
        points = self.link_data[self.current_link_index]["points"]
        current_point_index = get_closest_index_kdtree(points, self.ego_loc)

        if self.current_link_index == len(self.link_data) - 1:
            self.global_waypoints = points[current_point_index:]
            max_speed = self.link_data[self.current_link_index]["max_speed"]
            self.max_speed_pub.publish(max_speed)
            self.publish_waypoints(self.global_waypoints)
            return
        
        # Check if the current point index is the last or second last point
        if len(points) - current_point_index <= 2:
            self.current_link_index += 1
            current_point_index = 0

        current_link_index = self.current_link_index
        points = self.link_data[current_link_index]["points"]
        print("current_link_index",current_link_index)

        while len(self.global_waypoints) < 40:
            needed_points_num = 40 - len(self.global_waypoints)
            remained_points_num_in_current_link = len(points) - current_point_index

            if remained_points_num_in_current_link <= needed_points_num:
                self.global_waypoints += points[current_point_index:]
                self.current_link_index += 1

                if self.current_link_index >= len(self.link_data):
                    break

                current_point_index = 0
                points = self.link_data[self.current_link_index]["points"] 
            else:
                self.global_waypoints += points[current_point_index:current_point_index + needed_points_num]

        self.publish_waypoints(self.global_waypoints)

        max_speed = self.link_data[self.current_link_index]["max_speed"]
        self.max_speed_pub.publish(max_speed)


    def publish_waypoints(self, waypoints: list):
        waypoints_pub = Path()

        for waypoint in waypoints:
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = waypoint[0]
            tmp_pose.pose.position.y = waypoint[1]
            tmp_pose.pose.position.z = 0
            tmp_pose.pose.orientation.x = 0
            tmp_pose.pose.orientation.y = 0
            tmp_pose.pose.orientation.z = 0
            tmp_pose.pose.orientation.w = 1
            waypoints_pub.poses.append(tmp_pose)

        waypoints_pub.header.frame_id = "map"

        self.waypoints_pub.publish(waypoints_pub)
        self.link_index.data = self.current_link_index
        self.linkindex_pub.publish(self.link_index)
        self.

    def odom_callback(self, odom_msg):
        self.is_odom = True
        self.ego_loc = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]

if __name__ == '__main__':
    try:
        waypoint_parser = WaypointParser()
    except rospy.ROSInterruptException:
        pass
