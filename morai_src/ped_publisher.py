#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
import torch
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from vision_msgs.msg import BoundingBox3DArray, BoundingBox3D
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import time
from scipy.spatial import KDTree
from tf import TransformListener
import tf2_ros
import tf2_geometry_msgs
import open3d as o3d
from math import atan2, sqrt, pow, cos, sin
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PointStamped
import copy

PEDESTRIAN = 0.
CARGOBOX = 1.
PEDRUM = 2.

class PedestrianDetection:
    def __init__(self):
        self.bridge = CvBridge().compressed_imgmsg_to_cv2
        self.result_publish = rospy.Publisher("/detection_result", String, queue_size=10)
        self.bbox3d_publish = rospy.Publisher("/whole_clusters", BoundingBox3DArray, queue_size=10)
        #self.point_viz = rospy.Publisher("/point_viz", PointCloud2, queue_size=10)
        self.visualization_publish = rospy.Publisher("/yolo_viz", MarkerArray, queue_size=10)
        # 카메라 이미지 구독
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback, queue_size=10)
        self.velodyne_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.velodyne_callback)
        self.detections_sub = rospy.Subscriber("/detections", BoundingBox3DArray, self.bbox_callback)
        self.model = YOLO('/home/mobilion/catkin_ws/src/aMAP/src/models/ped_1002.pt')
        self.img_bgr = None
        self.detections = BoundingBox3DArray()
        self.other_detections = BoundingBox3DArray()
        self.final_detections = BoundingBox3DArray()
        self.visualization = MarkerArray()
        self.detections.header.frame_id = "gps"
        self.points = None
        self.frustum_points = None
        self.image_points = None
        self.frame_counter = 0
        self.frame_interval = 1  # YOLO 실행 간격(딜레이 방지)
        self.is_published = False
        self.is_from_pp = False
        self.last_published_time = time.time()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.K = np.array([[1000.0, 0, 640.0], [0, 1000.0, 360.0], [0, 0, 1]], dtype=np.float32) # fuck ...
        self.R = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=np.float32)

        self.LiDAR_2_world_homogenuous = np.array([[1, 0, 0, 1.31],[0, 1, 0, 0],[0, 0, 1, 1.60],[0, 0, 0, 1]], dtype=np.float32)
        self.cam_2_world_homogenuous = np.array([[1, 0, 0, 3.0],[0, 1, 0, 0],[0, 0, 1, 0.8], [0, 0, 0, 1]], dtype=np.float32)
        self.world_2_cam_homogenuous = np.linalg.inv(self.cam_2_world_homogenuous)
        self.LiDAR_2_cam_homogenuous = self.world_2_cam_homogenuous @ self.LiDAR_2_world_homogenuous
        self.GPS_2_world_homogenuous = np.array([[1, 0, 0, 3.0], [0, 1, 0, -0.1], [0, 0, 1, 0.47], [0, 0, 0, 1]], dtype=np.float32)
        self.LiDAR_2_GPS_homogenuous = np.linalg.inv(self.GPS_2_world_homogenuous) @ self.LiDAR_2_world_homogenuous

        self.listener = TransformListener()
        self.bboxes = []
        self.detections.header.frame_id = "gps"
        self.final_detections.header.frame_id = "gps"
        self.other_detections.header.frame_id = "gps"

    def transform_bounding_boxes(self, input_bboxes, target_frame):
        # 변환된 바운딩 박스들을 담을 새로운 BoundingBox3DArray 메시지 생성
        transformed_bboxes = BoundingBox3DArray()
        transformed_bboxes.header = input_bboxes.header
        transformed_bboxes.header.frame_id = target_frame

        for bbox in input_bboxes.boxes:
            # BoundingBox3D의 중심 좌표를 변환하기 위해 PointStamped 생성
            point_in = PointStamped()
            point_in.header.frame_id = input_bboxes.header.frame_id
            point_in.header.stamp = rospy.Time(0)  # 최신 TF 사용
            point_in.point = bbox.center.position  # 바운딩 박스의 중심 좌표

            try:
                # 중심 좌표를 target_frame으로 변환
                point_out = self.tf_buffer.transform(point_in, target_frame, rospy.Duration(1.0))

                # 변환된 좌표를 적용할 새 BoundingBox3D 생성
                transformed_bbox = BoundingBox3D()
                transformed_bbox = bbox  # 기존 bbox를 복사

                # 변환된 중심 좌표를 새 바운딩 박스에 적용
                transformed_bbox.center.position = point_out.point

                # 변환된 바운딩 박스를 결과 리스트에 추가
                transformed_bboxes.boxes.append(transformed_bbox)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"Failed to transform bounding box: {e}")

        return transformed_bboxes

    def numpy_to_pointcloud2(self):
        # 헤더 설정
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"

        # 포인트 필드 정의 (x, y, z)
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]

        # PointCloud2 메시지 생성
        pointcloud_msg = pc2.create_cloud(header, fields, self.frustum_points)
        #self.point_viz.publish(pointcloud_msg)

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def velodyne_callback(self, msg):
        points = self.pointcloud2_to_xyz(msg)
        self.voxel_grid_filter(points, leaf_size=0.1)
        self.apply_frustum_roi_filter()
        self.projection()
        self.find_depth_and_publish()

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
        roi_max_x = 25.0

        # Calculate the frustum limits for y-axis based on x and FOV
        roi_min_z = -2.25  # No lower limit for z
        roi_max_z = 0.0  # Remove all positive z values

        ego_x_max = 1.69 + 0.3
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
            self.numpy_to_pointcloud2()
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

    def find_depth_and_publish(self):
        if self.img_bgr is not None:
            self.frame_counter += 1
            if self.frame_counter % self.frame_interval == 0:
                res = self.model.predict(self.img_bgr)
                plots = res[0].plot()
                detected_ped = False
                detected_cbox = False
                detected_drum = False

                self.trackers = []
                self.bboxes = []

                if (len(res[0].boxes) > 0):

                    for i,box in enumerate(res[0].boxes):
                        cls = box.cls.item()

                        #tracker = cv2.TrackerKCF_create()
                        #tracker.init(self.img_bgr, box.xywh)
                        #self.trackers.append(tracker)
                        #self.bboxes.append(box.xywh)

                        if cls == PEDESTRIAN:
                            detected_ped = True
                            self.BBox3D_create(box, self.image_points, i)
                        elif cls == CARGOBOX:
                            detected_cbox = True
                            self.BBox3D_create(box, self.image_points, i)
                        elif cls == PEDRUM:
                            detected_drum = True
                            self.BBox3D_create(box, self.image_points, i)

                    if detected_ped:
                        self.signal_publish("ped")
                        detected_ped = False

                    if self.is_from_pp:
                        self.final_detections.boxes.extend(copy.deepcopy(self.detections.boxes))
                        self.final_detections.boxes.extend(copy.deepcopy(self.other_detections.boxes))
                        # self.detections.boxes.extend(self.other_detections.boxes)

                    self.detections.header.stamp = rospy.Time.now()
                    self.final_detections.header.stamp = rospy.Time.now()

                    if len(self.bboxes) <= 15:
                        transformed_bboxes = self.transform_bounding_boxes(self.detections, "map")
                        self.bboxes.append(copy.deepcopy(transformed_bboxes))  # 복사 후 추가
                    else:
                        self.bboxes.pop(0)
                        transformed_bboxes = self.transform_bounding_boxes(self.detections, "map")
                        self.bboxes.append(copy.deepcopy(transformed_bboxes))  # 복사 후 추가


                else:
                    if len(self.bboxes) > 0:
                        bboxes = self.bboxes.pop()
                        transformed_bboxes_origin = self.transform_bounding_boxes(bboxes, "velodyne")
                        self.final_detections.boxes.extend(copy.deepcopy(self.other_detections.boxes))
                        self.final_detections.boxes.extend(copy.deepcopy(transformed_bboxes_origin))
                    else: pass
                bbox3d = copy.deepcopy(self.final_detections)
                bbox3d.header.stamp = rospy.Time.now()

                viz = copy.deepcopy(self.visualization)
                self.bbox3d_publish.publish(bbox3d)
                self.visualization_publish.publish(viz)
                print(f"total drum and box: {len(self.visualization.markers)}")
                self.visualization.markers.clear()
                self.detections.boxes.clear()
                self.final_detections.boxes.clear()

                for point in self.image_points:
                    x, y = int(point[0]), int(point[1])
                    if 0 <= x < plots.shape[1] and 0 <= y < plots.shape[0]:
                        cv2.circle(plots, (x, y), radius=3, color=(0, 255, 0), thickness=-1)

                cv2.imshow("Image window", plots)
                cv2.waitKey(1)

    def BBox3D_create(self, box, image_points, i):
        label = box.cls.item()
        center_2d_x = box.xywh[0, 0].item()  # (xmin + xmax) / 2
        center_2d_y = box.xywh[0, 1].item()
        kdtree = KDTree(image_points)
        _, index = kdtree.query([center_2d_x, center_2d_y])
        closest_point_3d = self.frustum_points[index]

        if abs(sqrt(pow(center_2d_x,2) + pow(center_2d_y,2)) - sqrt(pow(image_points[index, 0],2) + pow(image_points[index, 1],2))) > 10.0:
            print(f"too far! {center_2d_x}, {center_2d_y} / {image_points[index, 0]}, {image_points[index, 1]}")
            print(abs(sqrt(pow(center_2d_x,2) + pow(center_2d_y,2)) - sqrt(pow(image_points[index, 0],2) + pow(image_points[index, 1],2))))
            return

        BBox3D = BoundingBox3D()
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.id = i
        marker.lifetime = rospy.Duration(3.0)

        point_ = PointStamped()
        point_.header.frame_id = "velodyne"
        point_.header.stamp = rospy.Time(0)
        point_.point.x = closest_point_3d[0]
        point_.point.y = closest_point_3d[1]
        point_.point.z = closest_point_3d[2]
        transformed_point = self.listener.transformPoint("map", point_)

        yaw = atan2(closest_point_3d[1], closest_point_3d[0])
        quarternion = quaternion_from_euler(0, 0, yaw)

        if label == PEDESTRIAN:
            BBox3D.size.x = 0.7
            BBox3D.size.y = 0.7
            BBox3D.size.z = 1.7
            diagonal = sqrt(2) * 0.7
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

            marker.ns = "ped"
            marker.scale.x = 0.7
            marker.scale.y = 0.7
            marker.scale.z = 1.7
            diagonal = sqrt(2) * 0.7
            marker.pose.position.x = transformed_point.point.x + (diagonal/2)*cos(yaw)
            marker.pose.position.y = transformed_point.point.y + (diagonal/2)*sin(yaw)
            marker.pose.position.z = transformed_point.point.z
            marker.pose.orientation.x = quarternion[0]
            marker.pose.orientation.y = quarternion[1]
            marker.pose.orientation.z = quarternion[2]
            marker.pose.orientation.w = quarternion[3]
            # marker.scale.x = 0.7
            # marker.scale.y = 0.7
            # marker.scale.z = 1.7
            self.detections.boxes.append(BBox3D)
            self.visualization.markers.append(marker)

        elif label == CARGOBOX:
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
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.4
            diagonal = sqrt(2) * 1.0
            marker.pose.position.x = transformed_point.point.x + (diagonal/2)*cos(yaw)
            marker.pose.position.y = transformed_point.point.y + (diagonal/2)*sin(yaw)
            marker.pose.position.z = transformed_point.point.z
            marker.pose.orientation.x = quarternion[0]
            marker.pose.orientation.y = quarternion[1]
            marker.pose.orientation.z = quarternion[2]
            marker.pose.orientation.w = quarternion[3]
            # marker.scale.x = 1.0
            # marker.scale.y = 1.0
            # marker.scale.z = 0.4
            self.detections.boxes.append(BBox3D)
            self.visualization.markers.append(marker)

        elif label == PEDRUM:
            BBox3D.size.x = 0.5
            BBox3D.size.y = 0.5
            BBox3D.size.z = 0.7
            diagonal = sqrt(2) * 0.5
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
            marker.ns = "drum"
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.7
            diagonal = sqrt(2) * 0.5
            marker.pose.position.x = transformed_point.point.x + (diagonal/2)*cos(yaw)
            marker.pose.position.y = transformed_point.point.y + (diagonal/2)*sin(yaw)
            marker.pose.position.z = transformed_point.point.z
            marker.pose.orientation.x = quarternion[0]
            marker.pose.orientation.y = quarternion[1]
            marker.pose.orientation.z = quarternion[2]
            marker.pose.orientation.w = quarternion[3]
            # marker.scale.x = 0.5
            # marker.scale.y = 0.5
            # marker.scale.z = 0.7
            self.detections.boxes.append(BBox3D)
            self.visualization.markers.append(marker)

    def bbox_callback(self, msg):
        self.is_from_pp = True
        self.other_detections = msg

    def signal_publish(self, result):
        self.result_publish.publish(result)

if __name__ == '__main__':
    rospy.init_node('camera_image', anonymous=True)
    pd = PedestrianDetection()
    rospy.spin()