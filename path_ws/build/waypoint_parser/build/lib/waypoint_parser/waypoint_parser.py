#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial import KDTree
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import UInt8
from geometry_msgs.msg import PoseStamped
import json
from pyproj import Transformer

LIMIT_DISTANCE = 10.0
#이런 값들은 나중에 따로 모아서 정리하기

class WaypointParser():
    def __init__(self):
        super().__init__('waypoint_parser')

        # ROS 2 Subscriber & Publisher 설정
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.waypoints_pub = self.create_publisher(Path, '/waypoints', 10)
        self.linkindex_pub = self.create_publisher(UInt8, '/link_index', 10)

        self.ego_loc = []  # WGS84 (위도, 경도)
        self.ego_loc_utm = []  # UTM 좌표
        self.is_odom = False
        self.current_link_index = 0
        self.waypoints = Path()
        self.link_index = UInt8()
        self.global_waypoints = []

        # WGS84 → UTM 변환기 설정 (예제: UTM Zone 52N)
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32652", always_xy=True)

        path = "/home/path.json"
        #경로 맞춰서 지정하기
        with open(path, 'r') as file:
            self.point_data = json.load(file)
        
        self.process_links()

        # 주기적으로 실행할 타이머 (50ms마다 실행)
        self.create_timer(0.05, self.update_waypoints)

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
    
    def get_closest_index_kdtree(points: list, indices: list, ego_odom: list) -> int:
        tree = KDTree(points)
        dist, local_index = tree.query([ego_odom[0], ego_odom[1]])

        global_index = indices[local_index]  # 원래 point_data에서의 인덱스로 변환

        if dist > LIMIT_DISTANCE and index < len(points) - 1:
            return global_index
        else:
            return indices[min(local_index + 1, len(indices) - 1)]
        
    def wgs84_to_utm(self, lat, lon):
        """WGS84에서 UTM으로 변환"""
        utm_x, utm_y = self.transformer.transform(lon, lat)
        return utm_x, utm_y
    
    def update_waypoints(self):
        """주기적으로 실행하여 웨이포인트 업데이트 및 퍼블리시"""
        if not self.is_odom:
            return

        self.global_waypoints = []
        self.create_and_publish_global_waypoints()
        self.is_odom = False  # 갱신 후 초기화

    def create_and_publish_global_waypoints(self):
        if self.current_link_index not in self.link_data:
            return
        
        points = self.link_data[self.current_link_index]["points"]
        current_point_index = self.get_closest_index_kdtree(
            points,
            self.link_data[self.current_link_index]["point_index"],
            self.ego_loc_utm
        )

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
        waypoints_pub.header.stamp = self.get_clock().now().to_msg()

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
        lat = odom_msg.pose.pose.position.x  
        lon = odom_msg.pose.pose.position.y 

        # WGS84 좌표를 UTM으로 변환
        self.ego_loc_utm = self.wgs84_to_utm(lat, lon)

        # WGS84 위치도 저장 (추후에 확인 필요)
        self.ego_loc = [lat, lon]

        self.get_logger().info(f"Converted UTM: ({self.ego_loc_utm[0]}, {self.ego_loc_utm[1]})")


def main(args=None):
    rclpy.init(args=args)
    waypoint_parser = WaypointParser()
    rclpy.spin(waypoint_parser)
    waypoint_parser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
