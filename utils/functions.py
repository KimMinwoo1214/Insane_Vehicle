#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from scipy.spatial import KDTree
from typing import Any, Tuple
from math import cos, sin

MINIMUM_TURNING_RADIUS = 5.97 #수정해야함
MAXIMUM_CURVATURE = pow(MINIMUM_TURNING_RADIUS, -1)

def config_loader(target_data_key: str) -> Any:
    with open("/home/mobilion/Desktop/semi_planning_control/src/src//configs/configs.yaml") as f: #파일경로 수정해야함
        config = yaml.load(f, Loader=yaml.FullLoader)
    target_data = config[target_data_key]
    return target_data

def calc_curvature(local_path_points):
    #curvature = Float32()

    x = np.array([local_path_point.pose.position.x for local_path_point in local_path_points.poses])
    y = np.array([local_path_point.pose.position.y for local_path_point in local_path_points.poses])

    # 3rd order polynomial fitting
    z = np.polyfit(x, y, 3)
    p = np.poly1d(z)

    # calculate curvature
    first_derivative = p.deriv()
    second_derivative = first_derivative.deriv()
    curvature_list = [min(MAXIMUM_CURVATURE,(abs(second_derivative(x[i]) / (1 + first_derivative(x[i]) ** 2) ** 1.5))) for i in range(len(local_path_points.poses))]

    index = np.argmax(curvature_list)
    curvature = curvature_list[index]
    return curvature

def calc_curvature_and_slope(local_path_points):
    x = np.array([local_path_point.pose.position.x for local_path_point in local_path_points.poses])
    y = np.array([local_path_point.pose.position.y for local_path_point in local_path_points.poses])

    # 3rd order polynomial fitting
    z = np.polyfit(x, y, 3)
    p = np.poly1d(z)

    # calculate curvature
    first_derivative = p.deriv()
    second_derivative = first_derivative.deriv()
    # curvature_list = [min(MAXIMUM_CURVATURE,(abs(second_derivative(x[i]) / (1 + first_derivative(x[i]) ** 2) ** 1.5))) for i in range(len(local_path_points.poses))]
    # curvature_list = [min(MAXIMUM_CURVATURE,(abs(second_derivative(x[i]) / ((1 + first_derivative(x[i]) ** 2) ** 1.5)))) for i in range(15)]
    curvature_list = [min(1e3,(abs(second_derivative(x[i]) / ((1 + first_derivative(x[i]) ** 2) ** 1.5)))) for i in range(len(local_path_points.poses))]

    index = np.argmax(curvature_list)
    curvature = curvature_list[index]
    return first_derivative, curvature

# waypoint_parser.py
def get_closest_index_kdtree(points: list, ego_odom: list) -> int:
    points_xy = np.array(points)[:, :2]
    tree = KDTree(points_xy)
    dist, index = tree.query([ego_odom[0], ego_odom[1]])

    if dist > config_loader("LIMIT_DISTANCE") and index < len(points) - 1:
        return index
    else:
        return index + 1

def calb_tangent(euler: Tuple[float, float, float], acc: np.ndarray) -> np.ndarray:
    roll, pitch, _ = euler
    gravity = np.array([
        -np.sin(pitch),
        np.sin(roll) * np.cos(pitch),
        np.cos(roll) * np.cos(pitch)
    ]) * 9.81

    return acc - gravity

def calc_vel(prev_x: float, prev_y: float, current_x: float, current_y: float, yaw: float) -> list:
    dt = 0.025 # 40Hz

    R = np.array([
    [cos(yaw), sin(yaw)],
    [-sin(yaw), cos(yaw)]
    ])

    global_velocity = np.array([(current_x - prev_x) / dt, (current_y - prev_y) / dt])

    local_velocity = R.dot(global_velocity)

    return [local_velocity[0], local_velocity[1]]

