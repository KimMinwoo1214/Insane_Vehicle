#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
import torch
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage, PointCloud2
from vision_msgs.msg import BoundingBox3DArray, BoundingBox3D
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import time
from scipy.spatial import KDTree

import open3d as o3d
from math import atan2,sqrt,pow,cos,sin
from tf.transformations import quaternion_from_euler

PEDESTRIAN = 0.
CARGOBOX = 1.
PEDRUM = 2.

class PedestrianDetection:
    def __init__(self):
        self.bridge = CvBridge()
        self.result_publish = rospy.Publisher("/detection_result", String, queue_size=50)
        self.bbox3d_publish = rospy.Publisher("/drum_or_cbox", BoundingBox3DArray, queue_size=10)
        self.visualization_publish = rospy.Publisher("/drum_cbox_viz", MarkerArray, queue_size=10)
        # 카메라 이미지 구독
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback, queue_size=10)
        self.velodyne_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.velodyne_callback)
        self.model = YOLO('/home/mobilion/catkin_ws/src/aMAP/src/models/ped_1002.pt')
        
        self.detections = BoundingBox3DArray()
        self.visualization = MarkerArray()
        self.detections.header.frame_id = "gps"
        self.points = None
        self.frustum_points = None
        self.image_points = None
        self.frame_counter = 0
        self.frame_interval = 1  # YOLO 실행 간격(딜레이 방지)
        self.is_published = False
        self.last_published_time = time.time()

        self.K = np.array([[1000, 0, 640.0], [0, 850, 360.0], [0, 0, 1]], dtype=np.float32) # fuck ...
        self.R = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=np.float32)

        self.LiDAR_2_world_homogenuous = np.array([[1, 0, 0, 1.31],[0, 1, 0, 0],[0, 0, 1, 1.60],[0, 0, 0, 1]], dtype=np.float32)
        self.cam_2_world_homogenuous = np.array([[1, 0, 0, 3.0],[0, 1, 0, 0],[0, 0, 1, 0.8], [0, 0, 0, 1]], dtype=np.float32)
        self.world_2_cam_homogenuous = np.linalg.inv(self.cam_2_world_homogenuous)
        self.LiDAR_2_cam_homogenuous = self.world_2_cam_homogenuous @ self.LiDAR_2_world_homogenuous
        self.GPS_2_world_homogenuous = np.array([[1, 0, 0, 3.0], [0, 1, 0, -0.1], [0, 0, 1, 0.47], [0, 0, 0, 1]], dtype=np.float32)
        self.LiDAR_2_GPS_homogenuous = np.linalg.inv(self.GPS_2_world_homogenuous) @ self.LiDAR_2_world_homogenuous

    def callback(self, msg):
        try:
            # 이미지 디코딩
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img_bgr is not None:
                self.frame_counter += 1
                if self.frame_counter % self.frame_interval == 0:
                    res = self.model.predict(img_bgr)
                    plots = res[0].plot()
                    # 클래스별로 결과 처리
                    detected_ped = False
                    detected_cbox = False
                    detected_drum = False
                    for box in res[0].boxes:
                        cls = box.cls.item()  # 클래스 ID
                        if cls == 0:  # 클래스 0: 보행자
                            detected_ped = True
                        elif cls == 1:  # 클래스 1: 장애물
                            detected_cbox = True
                            self.BBox3D_create(box, self.image_points)
                        elif cls == 2:
                            detected_drum = True
                            self.BBox3D_create(box, self.image_points)

                    # 결과 퍼블리시
                    if detected_ped:
                        self.signal_publish("ped")
                        detected_ped = False
                    # elif detected_cbox:
                    #     self.signal_publish("cbox")
                    # elif detected_drum:
                    #     self.signal_publish("drum")
                    # else:
                    #     self.signal_publish("none")
                    
                    self.detections.header.stamp = rospy.Time.now()
                    self.bbox3d_publish.publish(self.detections)
                    self.visualization_publish.publish(self.visualization)
                    print(f"total drum and box: {len(self.visualization.markers)}")
                    self.visualization.markers.clear()
                    self.detections.boxes.clear()
                

                    for point in self.image_points:
                        x, y = int(point[0]), int(point[1])  # 좌표를 정수로 변환
                        if 0 <= x < plots.shape[1] and 0 <= y < plots.shape[0]:  # 이미지 경계 내인지 확인
                            cv2.circle(plots, (x, y), radius=3, color=(0, 255, 0), thickness=-1)  # 초록색 점 그리기
                    cv2.imshow("Image window", plots)
                    cv2.waitKey(1)
                    
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def velodyne_callback(self, msg):
        # 포인트 클라우드 데이터를 Voxel Grid 필터로 다운샘플링
        points = self.pointcloud2_to_xyz(msg)
        self.voxel_grid_filter(points, leaf_size=0.1)
        self.apply_frustum_roi_filter()
        self.projection()

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            point_list.append((point[0], point[1], point[2]))
        point_np = np.array(point_list, np.float32)
        return point_np

    def voxel_grid_filter(self, points, leaf_size=0.1):
        # Voxel Grid 필터 적용
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 2. 복셀 그리드 필터 적용 (voxel downsampling)
        downsampled_pcd = pcd.voxel_down_sample(voxel_size=leaf_size)

        # 3. 결과를 NumPy 배열로 변환하여 반환
        self.points = np.asarray(downsampled_pcd.points)

    def apply_frustum_roi_filter(self):
        # FOV in degrees
        fov_deg = 65
        fov_rad = np.radians(fov_deg / 2)  # Half FOV in radians

        # x-axis range (forward direction)
        roi_min_x = 3.0
        roi_max_x = 50.0

        # Calculate the frustum limits for y-axis based on x and FOV
        roi_min_z = -1.9  # No lower limit for z
        roi_max_z = 0.0  # Remove all positive z values

        ego_x_max = 1.69 + 0.35
        ego_x_min = -2.945
        ego_y_max = 1.892/2
        ego_y_min = -1.892/2

        if self.points.size > 0:
            roi_mask = (
                (self.points[:, 0] >= roi_min_x) & (self.points[:, 0] <= roi_max_x) &
                (self.points[:, 2] >= roi_min_z) & (self.points[:, 2] <= roi_max_z) &
                # Y limits are determined by the FOV
                (np.abs(self.points[:, 1]) <= self.points[:, 0] * np.tan(fov_rad))
            )
            ego_mask = (
                (self.points[:, 0] >= ego_x_min) & (self.points[:, 0] <= ego_x_max) &
                (self.points[:, 1] >= ego_y_min) & (self.points[:, 1] <= ego_y_max)
            )
            mask = roi_mask & ~ego_mask
            self.frustum_points = self.points[mask]
        else: rospy.logwarn("there is no self.point")

    def projection(self):
        lidar_points_homogenuous = np.hstack((self.frustum_points, np.ones((self.frustum_points.shape[0], 1))))
        world_points_homogenuous = self.LiDAR_2_world_homogenuous @ lidar_points_homogenuous.T
        world_points = (world_points_homogenuous[:3,:]/world_points_homogenuous[3,:]).T
        new_world_homogenuous = np.hstack((world_points, np.ones((world_points.shape[0], 1))))
        camera_points_homogenuous = self.world_2_cam_homogenuous @ new_world_homogenuous.T
        camera_points = self.R @ (camera_points_homogenuous[:3, :] / camera_points_homogenuous[3, :])
        image_points_homogenuous = self.K @ camera_points
        self.image_points = (image_points_homogenuous[:2, :] / (image_points_homogenuous[2, :])).T

    def BBox3D_create(self, box, image_points):
        label = box.cls.item()
        center_2d_x = box.xywh[0, 0].item()  # (xmin + xmax) / 2
        center_2d_y = box.xywh[0, 1].item()
        kdtree = KDTree(image_points)
        _, index = kdtree.query([center_2d_x, center_2d_y])
        closest_point_3d = self.frustum_points[index]

        if abs(sqrt(pow(center_2d_x,2) + pow(center_2d_y,2)) - sqrt(pow(image_points[index, 0],2) + pow(image_points[index, 1],2))) > 5.0: 
            print(f"too far! {center_2d_x}, {center_2d_y} / {image_points[index, 0]}, {image_points[index, 1]}")
            print(abs(sqrt(pow(center_2d_x,2) + pow(center_2d_y,2)) - sqrt(pow(image_points[index, 0],2) + pow(image_points[index, 1],2))))
            return

        BBox3D = BoundingBox3D()
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "velodyne"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.lifetime = rospy.Duration(0.1)

        yaw = atan2(closest_point_3d[1], closest_point_3d[0])
        quarternion = quaternion_from_euler(0, 0, yaw)
        if label == CARGOBOX:
            BBox3D.size.x = 1.0
            BBox3D.size.y = 1.0
            BBox3D.size.z = 0.4
            diagonal = sqrt(2) * 1.0
            LiDAR_point = np.array([[closest_point_3d[0] + (diagonal/2)*cos(yaw)], [closest_point_3d[1] + (diagonal/2)*sin(yaw)], [closest_point_3d[2]], [1]], dtype=np.float32)
            gps_point_homogenuous = self.LiDAR_2_GPS_homogenuous @ LiDAR_point
            gps_point = gps_point_homogenuous[:3]/gps_point_homogenuous[3]
            BBox3D.center.position.x = gps_point[0].item()
            BBox3D.center.position.y = gps_point[1].item()
            BBox3D.center.position.z = gps_point[2].item()
            BBox3D.center.orientation.x = quarternion[0]
            BBox3D.center.orientation.y = quarternion[1]
            BBox3D.center.orientation.z = quarternion[2]
            BBox3D.center.orientation.w = quarternion[3]
            marker.ns = "cbox"
            marker.pose.position.x = LiDAR_point[0].item()
            marker.pose.position.y = LiDAR_point[1].item()
            marker.pose.position.z = LiDAR_point[2].item()
            marker.pose.orientation.x = quarternion[0]
            marker.pose.orientation.y = quarternion[1]
            marker.pose.orientation.z = quarternion[2]
            marker.pose.orientation.w = quarternion[3]
            marker.scale.x = BBox3D.size.x
            marker.scale.y = BBox3D.size.y
            marker.scale.z = BBox3D.size.z
            self.detections.boxes.append(BBox3D)
            self.visualization.markers.append(marker)

        elif label == PEDRUM:
            BBox3D.size.x = 0.6
            BBox3D.size.y = 0.6
            BBox3D.size.z = 0.8
            diagonal = sqrt(2) * 0.6
            LiDAR_point = np.array([[closest_point_3d[0] + (diagonal/2)*cos(yaw)], [closest_point_3d[1] + (diagonal/2)*sin(yaw)], [closest_point_3d[2]], [1]], dtype=np.float32)
            gps_point_homogenuous = self.LiDAR_2_GPS_homogenuous @ LiDAR_point
            gps_point = gps_point_homogenuous[:3]/gps_point_homogenuous[3]
            BBox3D.center.position.x = gps_point[0].item()
            BBox3D.center.position.y = gps_point[1].item()
            BBox3D.center.position.z = gps_point[2].item()
            BBox3D.center.orientation.x = quarternion[0]
            BBox3D.center.orientation.y = quarternion[1]
            BBox3D.center.orientation.z = quarternion[2]
            BBox3D.center.orientation.w = quarternion[3]
            marker.ns = "cbox"
            marker.pose.position.x = LiDAR_point[0].item()
            marker.pose.position.y = LiDAR_point[1].item()
            marker.pose.position.z = LiDAR_point[2].item()
            marker.pose.orientation.x = quarternion[0]
            marker.pose.orientation.y = quarternion[1]
            marker.pose.orientation.z = quarternion[2]
            marker.pose.orientation.w = quarternion[3]
            marker.scale.x = BBox3D.size.x
            marker.scale.y = BBox3D.size.y
            marker.scale.z = BBox3D.size.z
            self.detections.boxes.append(BBox3D)
            self.visualization.markers.append(marker)

    def signal_publish(self, result):
        self.result_publish.publish(result)

if __name__ == '__main__':
    rospy.init_node('camera_image', anonymous=True)
    pd = PedestrianDetection()
    rospy.spin()
