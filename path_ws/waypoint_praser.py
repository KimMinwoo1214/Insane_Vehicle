#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from scipy.spatial import KDTree
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import UInt8, Float32, Bool
from geometry_msgs.msg import PoseStamped
import json
import os
from utils.functions import get_closest_index_kdtree #파일명 수정 필요
import time

LIMIT_DISTANCE = 10.0
#이런 값들은 나중에 따로 모아서 정리하기

class WaypointParser():
    def __init__(self):
        rospy.init_node('waypoint_parser', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.waypoints_pub = rospy.Publisher('/waypoints', Path, queue_size=5)
        self.linkindex_pub = rospy.Publisher('/link_index', UInt8, queue_size=1)

        self.ego_loc = []
        self.is_odom = False
        self.current_link_index = 0
        self.waypoints = Path()
        self.link_index = UInt8()
        self.global_waypoints = []

        path = "/home/path.json"
        #경로 맞춰서 지정하기
        with open(path, 'r') as file:
            self.point_data = json.load(file)
        
        self.process_links()
        
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.is_odom:
                #start_time = time.time()
                self.global_waypoints = []
                self.create_and_publish_global_waypoints()
                self.is_odom = False
            rate.sleep()

    def process_links(self):
        """링크를 30개씩 묶어 저장."""
        self.link_data = {}
        link_size = 30
 
        for i in range(0, len(self.point_data), link_size):
            link_index = i // link_size
            self.link_data[link_index] = {
                "point_index":self.point_data[0][i:min(i + link_size, len(self.point_data[0]))],
                "points": list(zip(self.point_data[1][i:min(i + link_size, len(self.point_data[1]))],
                                   self.point_data[2][i:min(i + link_size, len(self.point_data[2]))]))
            }
    
    def get_closest_index_kdtree(points: list, ego_odom: list) -> int:
        points_xy = np.array(points)[:, :2]
        tree = KDTree(points_xy)
        dist, index = tree.query([ego_odom[0], ego_odom[1]])

        if dist > LIMIT_DISTANCE and index < len(points) - 1:
            return index
        else:
            return index + 1

    def create_and_publish_global_waypoints(self):
        if self.current_link_index not in self.link_data:
            return

        points = self.link_data[self.current_link_index]["points"]
        current_point_index = get_closest_index_kdtree(points, self.ego_loc)

        if len(points) - current_point_index <= 2:
            self.current_link_index += 1
            if self.current_link_index >= len(self.link_data):
                return
            points = self.link_data[self.current_link_index]["points"]
            current_point_index = 0

        while len(self.global_waypoints) < 40:
            needed_points_num = 40 - len(self.global_waypoints)
            remained_points_num_in_current_link = len(points) - current_point_index

            if remained_points_num_in_current_link <= needed_points_num:
                self.global_waypoints += points[current_point_index:]
                self.current_link_index += 1

                if self.current_link_index >= len(self.point_data):
                    break

                current_point_index = 0
                points = self.point_data[self.current_link_index]["points"] 
            else:
                self.global_waypoints += points[current_point_index:current_point_index + needed_points_num]
                break

        self.publish_waypoints(self.global_waypoints)

    def publish_waypoints(self, waypoints: list):
        waypoints_pub = Path()
        waypoints_pub.header.frame_id = "map"
        #좌표계 확인해서 넣기
        waypoints_pub.header.stamp = rospy.Time.now()

        for waypoint in waypoints:
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = waypoint[0]
            tmp_pose.pose.position.y = waypoint[1]
            tmp_pose.pose.position.z = 0
            tmp_pose.pose.orientation.w = 1
            waypoints_pub.poses.append(tmp_pose)

        self.waypoints_pub.publish(waypoints_pub)
        self.link_index.data = self.current_link_index
        self.linkindex_pub.publish(self.link_index)

    def odom_callback(self, odom_msg):
        self.is_odom = True
        self.ego_loc = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]

if __name__ == '__main__':
    try:
        waypoint_parser = WaypointParser()
    except rospy.ROSInterruptException:
        pass