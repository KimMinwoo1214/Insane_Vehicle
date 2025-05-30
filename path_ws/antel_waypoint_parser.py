#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import NavSatFix
from pyproj import Transformer
import csv

MODE = 1  # 1: GPS ON (use /fix), 2: GPS OFF (base_link 기준 상대 좌표)

class GPSBasedLocalPath(Node):
    def __init__(self):
        super().__init__('gps_based_local_path')

        self.local_path_pub = self.create_publisher(Path, '/local_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/local_path_markers', 10)

        if MODE == 1:
            self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)

        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32652", always_xy=True)
        self.global_path = self.load_csv_path('../Info/MongToGongcen.csv')

        self.utm_x, self.utm_y = None, None
        self.timer = self.create_timer(0.5, self.publish_path)

    # 서울 테스트용
    def load_csv_path(self, file_path):
        path = []
        with open(file_path, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                _, lon, lat = row
                utm_x, utm_y = self.transformer.transform(float(lon), float(lat))
                path.append((utm_x, utm_y))
        return path

    # 대회용
    # def load_csv_path(self, file_path):
    #     path = []
    #     with open(file_path, 'r', encoding='utf-8') as f:
    #         reader = csv.reader(f)
    #         next(reader)  # 헤더 스킵
    #         for row in reader:
    #             utm_x = float(row[3])  # UTM_X(East) 직접 사용
    #             utm_y = float(row[4])  # UTM_Y(North) 직접 사용
    #             path.append((utm_x, utm_y))
    #     return path

    def gps_callback(self, msg: NavSatFix):
        if msg.position_covariance_type != 1:
            self.get_logger().warn("Invalid GPS covariance type")
            return
        self.utm_x, self.utm_y = self.transformer.transform(msg.longitude, msg.latitude)

    def publish_path(self):
        if MODE == 1:
            if self.utm_x is None or self.utm_y is None:
                self.get_logger().warn("Waiting for GPS fix...")
                return
            local_path = self.build_local_path_from_position(self.utm_x, self.utm_y)
            local_path.header.frame_id = 'map'
        else:
            local_path = self.build_relative_path()
            local_path.header.frame_id = 'base_link'

        self.local_path_pub.publish(local_path)
        self.publish_markers(local_path)

    def build_local_path_from_position(self, cur_x, cur_y, horizon=30):
        path = Path()
        dists = [(i, (x - cur_x) ** 2 + (y - cur_y) ** 2) for i, (x, y) in enumerate(self.global_path)]
        nearest_index = sorted(dists, key=lambda x: x[1])[0][0]

        for (x, y) in self.global_path[nearest_index:nearest_index + horizon]:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def build_relative_path(self, distance_limit=10.0):
        path = Path()
        if not self.global_path:
            return path

        x0, y0 = self.global_path[0]
        total_dist = 0.0
        prev_x, prev_y = x0, y0

        for i, (x, y) in enumerate(self.global_path):
            rel_x = x - x0
            rel_y = y - y0

            dist = ((x - prev_x) ** 2 + (y - prev_y) ** 2) ** 0.5
            total_dist += dist
            prev_x, prev_y = x, y

            pose = PoseStamped()
            pose.pose.position.x = rel_x
            pose.pose.position.y = rel_y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

            if total_dist >= distance_limit:
                break

        return path

    def publish_markers(self, path: Path):
        marker_array = MarkerArray()
        for i, pose in enumerate(path.poses):
            marker = Marker()
            marker.header.frame_id = path.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = pose.pose.position
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = GPSBasedLocalPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
