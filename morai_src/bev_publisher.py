#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, UInt8, Bool
import os
from ultralytics import YOLO
from tf import TransformListener
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
import copy

class SCANParser:
    def __init__(self):
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.dist_pub = rospy.Publisher("/bev_bbox", Float32MultiArray, queue_size=10)
        self.bbox3d_car_pub = rospy.Publisher("/bev_with_state", Detection3DArray, queue_size=10)
        self.pc_np = None
        self.dist_pub_array = Float32MultiArray()

        self.model = YOLO("/home/mobilion/catkin_ws/src/aMAP/src/models/bev_1002.pt")
        self.frame_counter = 0
        self.frame_interval = 1  # YOLO를 실행할 프레임 간격(딜레이 방지)
        self.behavior = 0
        self.is_index = False

        self.listener = TransformListener()
        self.detections = Detection3DArray()
        self.last_detections = Detection3DArray()
        self.last_points = []
        self.current_points = []

    def callback(self, msg):
        self.frame_counter += 1  # 프레임 카운터 증가

        # 4 프레임마다 한 번씩 거리 계산, BEV 이미지 생성 및 객체 인식 수행
        if self.frame_counter % self.frame_interval == 0:

            self.pc_np = self.pointcloud2_to_xyz(msg)  # 포인트 클라우드를 xyz 데이터로 변환
            bev_image = self.generate_bev(self.pc_np)  # BEV 이미지 생성
            self.visualize_bev(bev_image)  # BEV 이미지 시각화

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        pointcloud = pc2.read_points(cloud_msg, skip_nans=True)
        for point in pointcloud:
            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = np.arctan2(point[1], point[0])
            if point[2] > -1.3 and dist < 35:
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))
        point_np = np.array(point_list, np.float32)
        return point_np

    def generate_bev(self, points):
        bev_image = np.zeros((1200, 1200, 3), dtype=np.uint8)  # 이미지 크기를 1200x1200으로 조정
        for point in points:
            x, y = point[0], point[1]
            # 스케일 조정 및 포인트 필터링 조건 변경
            if abs(x) < 25 and abs(y) < 25:  # LIDAR 포인트를 50x50m 범위로 제한
                pix_x = int((x + 25) * (1200 / 50))  # x 좌표를 픽셀로 변환 (이미지 크기 조정)
                pix_y = int((y + 25) * (1200 / 50))  # y 좌표를 픽셀로 변환 (이미지 크기 조정)
                # BEV 이미지에 포인트 그리기 (화이트), 원의 크기를 1로 조정
                cv2.circle(bev_image, (1200 - pix_y, 1200 - pix_x), 1, (255, 255, 255), -1)


        return bev_image

    def visualize_bev(self, bev_image):
        # now = datetime.now()
        # print(f"{now.day}d_{now.hour}h_{now.minute}m_{now.second}s.jpg")
        # cv2.imwrite(f"./image/{now.day}d_{now.hour}h_{now.minute}m_{now.second}s.jpg",bev_image)


        results = self.model(bev_image)

        rs= results[0].boxes.xywh.to("cpu")
        classes = results[0].boxes.cls
        rs_np = np.array(rs)

        # print(f"1:  {rs_np}")
        result_list = rs_np/24 - 25
        # print(f"2:  {result_list}")

        rs_list = result_list.tolist()

        detect_image = results[0].plot()

        float_array_msg = Float32MultiArray()

        # layout 설정
        float_array_msg.layout.dim.append(MultiArrayDimension())
        float_array_msg.layout.dim[0].label = "objects"
        float_array_msg.layout.dim[0].size = len(rs_list)
        float_array_msg.layout.dim[0].stride = len(rs_list) * 4  # 객체 수 * 각 객체의 속성 수

        float_array_msg.layout.dim.append(MultiArrayDimension())
        float_array_msg.layout.dim[1].label = "attributes"
        float_array_msg.layout.dim[1].size = 4
        float_array_msg.layout.dim[1].stride = 4  # x, y, w, h

        # 2차원 리스트를 1차원 리스트로 변환
        flat_list = [item for sublist in rs_list for item in sublist]
        float_array_msg.data = flat_list

        if float_array_msg.data is not None:
            for i in range(float_array_msg.layout.dim[0].size):
                detection = Detection3D()
                detection_result = ObjectHypothesisWithPose()
                detection_result.id = classes[i].item()
                detection.results.append(detection_result)
                point_ = PointStamped()
                point_.header.stamp = rospy.Time(0)
                point_.header.frame_id = "velodyne"
                point_.point.x = -flat_list[i*4+1]
                point_.point.y = -flat_list[i*4+0]
                point_.point.z = 1.0
                world_point = self.listener.transformPoint("map", point_)
                world_point_np = np.array([world_point.point.x, world_point.point.y, world_point.point.z])
                # print(f'last_points : {self.last_points}')
                # print(f'last_points_len : {len(self.last_points)}')
                if(len(self.last_points) > 0):
                    distances = [np.linalg.norm(world_point_np - np.array([p[0], p[1], p[2]])) for p in self.last_points]
                    min_index = np.argmin(distances)

                    if distances[min_index] < 0.5:
                        detection.results[0].score = 0.0 # 0.0: static / 1.0: dynamic
                        print("static")

                    else:
                        detection.results[0].score = 1.0 # 0.0: static / 1.0: dynamic
                        print(f"dynamic : {distances[min_index]}")

                    detection.results[0].id = 0
                    detection.bbox.center.position.x = point_.point.x
                    detection.bbox.center.position.y = point_.point.y
                    detection.bbox.center.position.z = point_.point.z
                    detection.bbox.center.orientation.x = 0.0
                    detection.bbox.center.orientation.y = 0.0
                    detection.bbox.center.orientation.z = 0.0
                    detection.bbox.center.orientation.w = 1.0
                    detection.bbox.size.x = flat_list[i*4+2]
                    detection.bbox.size.y = flat_list[i*4+3]
                    detection.bbox.size.z = 3.0

                    self.detections.detections.append(detection)
                    self.current_points.append(world_point_np)
                else:
                    # print("last point nono")
                    self.current_points.append(world_point_np)

            # 메시지 publish
            self.dist_pub.publish(float_array_msg)
            # print("Before publish: ", len(self.detections.detections))
            self.bbox3d_car_pub.publish(self.detections)
            # print("After publish: ", len(self.detections.detections))
            self.last_detections = copy.deepcopy(self.detections)
            self.last_points = copy.deepcopy(self.current_points)
            self.detections.detections.clear()
            self.current_points.clear()

        cv2.imshow("BEV", detect_image)

        cv2.waitKey(1)  # Refreshes the window

if __name__ == '__main__':
    rospy.init_node('velodyne_parser', anonymous=True)
    scan_parser = SCANParser()
    rospy.spin()

