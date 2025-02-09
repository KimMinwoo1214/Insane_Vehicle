#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path, Odometry
# from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8, Float32MultiArray, String, Bool
from scipy.optimize import minimize
import time
from vision_msgs.msg import BoundingBox3DArray, BoundingBox3D

# Behavior Macro
KEEPING_WAYPOINT = 0
EMERGENCY_STOP = 1
YIELD_TO_VEHICLE = 2 # ACC
LATTICE_PLAN = 3
TRAFFIC_STOP = 4

class BehaviorPlanner():
    def __init__(self):
        # initialize the node
        rospy.init_node('behavior_planner', anonymous=True)
        
        # subscribe the topics
        self.detected_objects_sub = rospy.Subscriber('/bev_bbox', Float32MultiArray, self.detected_objects_callback)
        self.local_path_sub = rospy.Subscriber('/local_path', Path, self.local_path_callback)
        self.link_index = rospy.Subscriber('/link_index', UInt8, self.link_index_callback)
        self.ped_sub = rospy.Subscriber('/ped_result', String, self.ped_callback)
        self.enter_blackout_sub = rospy.Subscriber('/enter_blackout', Bool, self.enter_blackout_callback)
        self.traffic_sub = rospy.Subscriber('/traffic_res', String, self.traffic_callback)
        self.detections_sub = rospy.Subscriber('detections', BoundingBox3DArray, self.cluster_callback)
        # publish the topic
        self.behavior_pub = rospy.Publisher('/behavior', UInt8, queue_size=100)
        
        # initialize the variables
        # self.current_position = PoseStamped()
        # self.detected_objects = Float32MultiArray()
        self.detected_car_array = []
        self.local_path = Path()
        self.link_index = 0
        self.ped = None
        self.traffic_color = ''
        self.behavior = UInt8()
        self.cluster_detections = BoundingBox3DArray()
        self.state = KEEPING_WAYPOINT
        
        self.is_detected_objects = False
        self.is_local_path = False
        self.is_link_index = False
        self.is_launch = False
        self.is_launch_arrived = False
        self.is_traffic = False
        self.is_ped = False
        self.is_enter_blackout = False
        self.is_waiting_first = True
        self.is_waiting_second = True
        
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            if self.is_local_path and self.is_link_index:
                self.behavior.data = self.find_new_state()
                self.behavior_pub.publish(self.behavior)
                print("behavior : ", self.behavior)
            else:
                pass

    def find_new_state(self):
        # check the gps is blackout
        if self.is_launch and self.is_launch_arrived:
            self.is_launch = False
            return EMERGENCY_STOP
        
        if self.is_ped and self.ped == "ped" :#and not 11 <= self.link_index <= 12:
            self.ped = ""
            return EMERGENCY_STOP
        
        if self.link_index == 30: # and self.is_traffic:
            if (self.traffic_color == "red") or (self.traffic_color == 'yellow'):
                return TRAFFIC_STOP
        
        # if the detected objects, check the object type and distance
        if self.is_detected_objects:
            self.is_detected_objects = False
            for obj in self.cluster_detections.boxes:
                # print("x : ", obj[0])
                # print("y : ", obj[1])
                # if (self.link_index == 2 or self.link_index == 20) and 0 < obj[0] < 7.5 and obj[1] > 0: # this index
                #     dist = self.calculate_distance(obj[0], obj[1])
                #     if dist < 12:
                #         print("NPC CAR IS ATTACKING EGO LANE")
                #         return EMERGENCY_STOP
                if self.is_on_lane(self.local_path, obj) and (self.behavior.data == KEEPING_WAYPOINT or self.behavior.data == LATTICE_PLAN or self.behavior.data == YIELD_TO_VEHICLE):
                    print("NPC IS ON LANE.")
                    print(obj.center.position.x, obj.center.position.y)
                    dist = self.calculate_distance(obj.center.position.x, obj.center.position.y)
                    print(dist)
                    if dist < 20 :
                        return LATTICE_PLAN
                    
        # default behavior is KEEPING_WAYPOINT 
        # if all the conditions are not satisfied, return KEEPING_WAYPOINT    
        return KEEPING_WAYPOINT
                    
    @staticmethod
    def calculate_distance(x, y):
        return math.sqrt(pow(x, 2) + pow(y, 2))
                
    def is_on_lane(self, local_path , detected_object):
        # check if the vehicle is on the lane
        local_x = np.array([waypoint.pose.position.x for waypoint in local_path.poses])
        local_y = np.array([waypoint.pose.position.y for waypoint in local_path.poses])

        tolerance = 2.0 # it is equal to the lane offset (half of the lane width)
        
        # fit the qubic spline to the local path
        local_path_coeff = np.polyfit(local_x, local_y, 3)
        local_path_poly = np.poly1d(local_path_coeff)
        
        x0, y0 = detected_object.center.position.x, detected_object.center.position.y
        
        def distance_squared(xc):
            yc = local_path_poly(xc)
            return (xc - x0)**2 + (yc - y0)**2
        
        result = minimize(distance_squared, x0, method='L-BFGS-B', bounds=[(min(local_x), max(local_x))])
        xc = result.x[0]
        yc = local_path_poly(xc)

        # calculate h
        h = np.sqrt((xc - x0)**2 + (yc - y0)**2)
        
        if h <= 3.5:
            return True
        
        return False

    def cluster_callback(self, msg):
        self.cluster_detections = msg
        self.is_detected_objects = True

    def detected_objects_callback(self, data):
        num_objects = data.layout.dim[0].size
        num_attributes = data.layout.dim[1].size
        flat_array = np.array(data.data)
        reshaped_array = flat_array.reshape((num_objects, num_attributes))

        if len(reshaped_array):
            for i in range(len(reshaped_array)):
                # reshaped_array[i][0] -= 25
                # reshaped_array[i][1] -= 25
                x = reshaped_array[i][1]
                y = reshaped_array[i][0]
                reshaped_array[i][0] = -(x - 25)
                reshaped_array[i][1] = -(y - 25)
                print(reshaped_array[i][0], reshaped_array[i][1])

        self.detected_car_array = reshaped_array
        
        if len(self.detected_car_array) >= 1:
            self.is_detected_objects = True
        
    def local_path_callback(self, msg):
        self.local_path = msg
        self.is_local_path = True
        
    def link_index_callback(self, msg):
        self.link_index = msg.data
        self.is_link_index = True
        
    def ped_callback(self, msg):
        self.ped = msg.data
        self.is_ped = True

    def enter_blackout_callback(self, msg):
        self.is_launch = msg.data
        self.is_launch_arrived = True

    def traffic_callback(self, msg):
        self.traffic_color = msg.data
        self.is_traffic = True

    def collision_check(self, ped):
        pass

if __name__ == '__main__':
    try:
        test_track = BehaviorPlanner()
    except rospy.ROSInterruptException:
        pass