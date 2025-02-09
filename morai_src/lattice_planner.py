#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from math import sqrt, cos, sin
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8, Float32MultiArray, Bool
from tf.transformations import euler_from_quaternion
from vision_msgs.msg import BoundingBox3DArray, BoundingBox3D
from visualization_msgs.msg import Marker, MarkerArray
import time

LATTICE_PLAN = 3

def compute_edges(data):
    x_center = data.center.position.x
    y_center = data.center.position.y
    box_size_x = data.size.x
    box_size_y = data.size.y
    box_orient = (data.center.orientation.x, data.center.orientation.y, data.center.orientation.z, data.center.orientation.w)
    _, _, box_yaw = euler_from_quaternion(box_orient)

    corners = np.array([
        [-box_size_x / 2, -box_size_y / 2],
        [ box_size_x / 2, -box_size_y / 2],
        [ box_size_x / 2,  box_size_y / 2],
        [-box_size_x / 2,  box_size_y / 2]
    ])

    rotation_matrix = np.array([
        [np.cos(box_yaw), -np.sin(box_yaw)],
        [np.sin(box_yaw),  np.cos(box_yaw)]
    ])
    
    edges = []
    markerarray = MarkerArray()

    for corner in corners:
        marker = Marker()
        rotated_corner = np.dot(rotation_matrix, corner)
        transformed_corner = rotated_corner + np.array([x_center, y_center])
        edges.append(transformed_corner)
        marker.header.frame_id = "gps"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(0.1)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.7
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.pose.position.x = transformed_corner[0]
        marker.pose.position.y = transformed_corner[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        markerarray.markers.append(marker)
        print(len(markerarray.markers))
        
    return edges, markerarray



class lattice_planner():
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)
        self.local_path_sub = rospy.Subscriber('/local_path', Path, self.local_path_callback)
        self.behavior_sub = rospy.Subscriber('/behavior', UInt8, self.behavior_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cluster_sub = rospy.Subscriber('whole_clusters', BoundingBox3DArray, self.cluster_callback)
        self.is_lastlane_sub = rospy.Subscriber('/is_last_lane', Bool, self.is_last_lane_callback)
        self.lattice_path_pub = rospy.Publisher('/lattice_path', Path, queue_size=1)
        ####
        self.viz_pub = rospy.Publisher("/corners", MarkerArray, queue_size=10)
        self.viz = MarkerArray()
        ####
        self.odom = Odometry()
        self.local_path = Path()
        self.behavior = UInt8()
        self.detected_objects = BoundingBox3DArray()
        self.lattice_path = Path()
        self.lattice_path_list = []

        self.is_odom = False
        self.is_detected_objects = False
        self.is_local_path = False
        self.is_behavior = False
        self.is_last_lane = True
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.is_local_path and self.is_detected_objects:
                if self.behavior.data == LATTICE_PLAN:
                    self.lattice_path_list = self.lattice_planning(self.local_path)
                    self.lattiice_path_index = self.collision_check(self.detected_objects, self.lattice_path_list)
                    start_time = time.time()
                    while time.time() - start_time < 0.3:
                        self.lattice_path_pub.publish(self.lattice_path_list[self.lattiice_path_index])
                        time.sleep(0.1)
                    # if self.is_last_lane:
                    #     while time.time() - start_time < 0.6:
                    #         self.lattice_path_pub.publish(self.lattice_path_list[self.lattiice_path_index])
                    #         time.sleep(0.1)                       
                    # else:
                    #     while time.time() - start_time < 0.95:
                    #         self.lattice_path_pub.publish(self.lattice_path_list[self.lattiice_path_index])
                    #         time.sleep(0.1)
                    self.is_odom = self.is_local_path = self.is_behavior = self.is_detected_objects = False
                else:
                    self.lattice_path_pub.publish(self.local_path)
            rate.sleep()


    def collision_check(self, object_data, lattice_path):
        lane_weight = [1, 0, 30, 50, 80]

        if self.is_last_lane:
            lane_weight[0] += float('inf')

        for obstacle in object_data.boxes:
            edges, markerarray = compute_edges(obstacle)
            self.viz_pub.publish(markerarray)
            # TODO : Check every point whether collisions or not
            for path_num in range(len(lattice_path)):
                for i, path_pos in enumerate(lattice_path[path_num].poses):
                    for edge in edges:
                        dis = sqrt(pow(edge[0] - path_pos.pose.position.x, 2) +
                                    pow(edge[1] - path_pos.pose.position.y, 2))
                        if dis < 2:
                            print(f'path num : {path_num}, path_pos_num : {i}')
                            lane_weight[path_num] += 100

        print(f'lane weight : {lane_weight}')
        return lane_weight.index(min(lane_weight))

    def lattice_planning(self, ref_local_path):
        out_path = []
        # lattice_path.header.frame_id = 'odom'

        lane_offset_list = [-4, 0, 4, 5.5, 7]
        local_lattice_points = [[ref_local_path.poses[-1].pose.position.x, ref_local_path.poses[-1].pose.position.y + lane_offset_list[i], 1] for i in range(len(lane_offset_list))]

        for end_point in local_lattice_points:
            lattice_path = Path()
            lattice_path.header.frame_id = 'base_link'
            x_interval = 0.5
            xs = 0
            xf = end_point[0]
            ps = 0
            pf = end_point[1]
            x_num = xf / x_interval
            x = [i * x_interval for i in range(xs, int(x_num))]
            a = [0.0, 0.0, 0.0, 0.0]
            a[0] = ps
            a[1] = 0
            a[2] = 3.0 * (pf - ps) / pow(xf, 2)
            a[3] = -2.0 * (pf - ps) / pow(xf, 3)

            y = [a[3] * pow(i, 3) + a[2] * pow(i, 2) + a[1] * i + a[0] for i in x]

            for i in range(len(x)):
                pose = PoseStamped()
                pose.pose.position.x = x[i]
                pose.pose.position.y = y[i]
                pose.pose.position.z = 0
                pose.pose.orientation.x = 1
                lattice_path.poses.append(pose)

            out_path.append(lattice_path)

        return out_path

    def odom_callback(self, msg):
        self.odom = msg
        self.is_odom = True

    def cluster_callback(self, msg):
        self.detected_objects = msg
        self.is_detected_objects = True

    def local_path_callback(self, msg):
        self.local_path = msg
        self.is_local_path = True

    def behavior_callback(self, msg):
        self.behavior = msg
        self.is_behavior = True

    def is_last_lane_callback(self, msg):
        self.is_last_lane = msg.data


if __name__ == '__main__':
    try:
        test_track = lattice_planner()
    except rospy.ROSInterruptException:
        pass