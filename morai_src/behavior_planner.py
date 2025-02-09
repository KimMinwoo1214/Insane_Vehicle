#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from math import sqrt
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path, Odometry
# from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8, String, Bool
from vision_msgs.msg import BoundingBox3DArray, Detection3DArray
from scipy.optimize import minimize
import time

KEEPING_WAYPOINT = 0
EMERGENCY_STOP = 1 # When the pedestrian is detected
YIELD_TO_VEHICLE = 2 # ACC
LATTICE_PLAN = 3
TRAFFIC_STOP = 4
FINISH = 5

traffic_index_list = [5, 19]
traffic_stop_line_y_list = [-423.5, -100]

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

    for corner in corners:
        rotated_corner = np.dot(rotation_matrix, corner)
        transformed_corner = rotated_corner + np.array([x_center, y_center])
        edges.append(transformed_corner)

    return edges

class BehaviorPlanner():
    def __init__(self):
        # initialize the node
        rospy.init_node('behavior_planner', anonymous=True)
        # subscribe the topics
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.detected_objects_sub = rospy.Subscriber('/bev_bbox', Detection3DArray, self.detected_objects_callback)
        self.local_path_sub = rospy.Subscriber('/local_path', Path, self.local_path_callback)
        self.link_index = rospy.Subscriber('/link_index', UInt8, self.link_index_callback)
        self.ped_sub = rospy.Subscriber('/ped_result', String, self.ped_callback)
        self.traffic_sub = rospy.Subscriber('/traffic_res', String, self.traffic_callback)
        self.cluster_sub = rospy.Subscriber('whole_clusters', BoundingBox3DArray, self.cluster_callback)
        self.cluster2_sub = rospy.Subscriber('detections', BoundingBox3DArray, self.cluster2_callback)
        # publish the topic
        self.behavior_pub = rospy.Publisher('/behavior', UInt8, queue_size=100)

        # initialize the variables
        self.odom = Odometry()
        # self.current_position = PoseStamped()
        # self.detected_objects = Float32MultiArray()
        self.detected_car_array = []
        self.detected_objects = BoundingBox3DArray()
        self.detected_objects2 = BoundingBox3DArray()
        self.local_path = Path()
        self.link_index = 0
        self.current_position_x = 0
        self.current_position_y = 0
        self.ped = None
        self.traffic_color = ''
        self.behavior = UInt8()
        self.state = KEEPING_WAYPOINT

        self.is_odom = False
        self.is_detected_objects = False
        self.is_local_path = False
        self.is_link_index = False
        self.is_traffic = False
        self.is_ped = False
        self.is_detected_cluster = False

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.is_odom and self.is_local_path and self.is_link_index:
                self.behavior.data = self.find_new_state()
                self.behavior_pub.publish(self.behavior)
                self.is_detected_objects = self.is_local_path = self.is_link_index = self.is_traffic = self.is_ped = self.is_odom = False
                rate.sleep()
            else:
                pass


    def find_new_state(self):
        if self.link_index == 29:
            return FINISH

        if self.is_ped and self.ped == "ped" :#and not 11 <= self.link_index <= 12:
            self.ped = ""
            return EMERGENCY_STOP

        if self.link_index in traffic_index_list: # and self.is_traffic:
            if (self.traffic_color == "red"):
                if self.link_index == 5:
                    if self.is_detected_cluster:
                        if self.is_collision(self.detected_objects, self.local_path):
                            return TRAFFIC_STOP
                    if self.current_position_y > traffic_stop_line_y_list[0]:
                        return TRAFFIC_STOP

                elif self.link_index == 19:
                    if self.is_detected_cluster:
                        if self.is_collision(self.detected_objects, self.local_path):
                            return TRAFFIC_STOP
                    if self.current_position_y > traffic_stop_line_y_list[1]:
                        return TRAFFIC_STOP

        # if the detected objects, check the object type and distance
        if self.detected_car_array is not None:
            for obj in self.detected_car_array:
                if self.is_on_lane(self.local_path, obj) and self.behavior.data == KEEPING_WAYPOINT and obj[0] > 0 and obj[2] == 1:
                    print("NPC CAR IS ON LANE.")
                    print(obj[0], obj[1])
                    dist = self.calculate_distance(obj[0], obj[1])
                    print(dist)
                    if dist < 20:
                        return YIELD_TO_VEHICLE

        if (self.detected_objects is not None) and not self.link_index == 19:
            if self.is_collision(self.detected_objects, self.local_path):
                return LATTICE_PLAN

        if (self.detected_objects2 is not None) and not self.link_index == 19:
            if self.is_collision(self.detected_objects2, self.local_path):
                return LATTICE_PLAN


        # default behavior is KEEPING_WAYPOINT
        # if all the conditions are not satisfied, return KEEPING_WAYPOINT
        return KEEPING_WAYPOINT

    @staticmethod
    def calculate_distance(x, y):
        return math.sqrt(pow(x, 2) + pow(y, 2))

    def transition_to(self, new_state):
        if self.state != new_state:
            self.state = new_state

    def is_on_lane(self, local_path , detected_object):
        # check if the vehicle is on the lane
        local_x = np.array([waypoint.pose.position.x for waypoint in local_path.poses])
        local_y = np.array([waypoint.pose.position.y for waypoint in local_path.poses])

        tolerance = 1.5 # it is equal to the lane offset (half of the lane width)

        # fit the qubic spline to the local path
        local_path_coeff = np.polyfit(local_x, local_y, 3)
        local_path_poly = np.poly1d(local_path_coeff)

        x0, y0 = detected_object[0], detected_object[1]

        def distance_squared(xc):
            yc = local_path_poly(xc)
            return (xc - x0)**2 + (yc - y0)**2

        result = minimize(distance_squared, x0, method='L-BFGS-B', bounds=[(min(local_x), max(local_x))])
        xc = result.x[0]
        yc = local_path_poly(xc)

        # calculate h
        h = np.sqrt((xc - x0)**2 + (yc - y0)**2)

        if h <= tolerance:
            return True

        return False

    def odom_callback(self, msg):
        self.is_odom = True
        # odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        # _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y


    def detected_objects_callback(self, data):
        self.detected_car_array = []
        for detection in data:
            temp_list = []
            temp_list.append(detection.bbox.center.position.x)
            temp_list.append(detection.bbox.center.position.y)
            temp_list.append(detection.results.score)
            self.detected_car_array.append(temp_list)

        if len(self.detected_car_array) >= 1:
            self.is_detected_objects = True

    def is_collision(self, object_data, local_path):
        for obstacle in object_data.boxes:
            edges = compute_edges(obstacle)
            print("collision check")
            for i, path_pos in enumerate(local_path.poses):
                for edge in edges:
                    dis = sqrt(pow(edge[0] - path_pos.pose.position.x, 2) +
                                pow(edge[1] - path_pos.pose.position.y, 2))
                    if dis < 2.0:
                        return True

                dis = sqrt(pow(obstacle.center.position.x - path_pos.pose.position.x, 2) +
                            pow(obstacle.center.position.y - path_pos.pose.position.y, 2))
                print(f'dis : {dis}, i : {i}')
                if dis < 2.0:
                    return True

        return False

    def local_path_callback(self, msg):
        self.local_path = msg
        self.is_local_path = True

    def link_index_callback(self, msg):
        self.link_index = msg.data
        self.is_link_index = True

    def ped_callback(self, msg):
        self.ped = msg.data
        self.is_ped = True

    def traffic_callback(self, msg):
        self.traffic_color = msg.data
        self.is_traffic = True

    def cluster_callback(self, msg):
        self.detected_objects = msg
    
    def cluster2_callback(self, msg):
        self.detected_objects2 = msg


    def calculate_distance_obj(self, edge):
        center_point = [(edge.left_upper.x + edge.right_upper.x) / 2, (edge.left_upper.y + edge.left_lower.y) / 2]
        return math.sqrt(pow(center_point[0], 2) + pow(center_point[1], 2))



if __name__ == '__main__':
    try:
        test_track = BehaviorPlanner()
    except rospy.ROSInterruptException:
        pass