#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image 메시지 타입
from cv_bridge import CvBridge, CvBridgeError  # ROS와 OpenCV 이미지 변환 패키지
import cv2  # OpenCV 라이브러리
import numpy as np
class ImagePublisher(Node):
    """
    ImagePublisher 클래스: ROS2 노드로, 내장 웹캠에서 영상을 캡처하여 'video_frames' 토픽으로 퍼블리시합니다.
    """

    def __init__(self):
        super().__init__('image_publisher')
        
        # 'video_frames' 토픽에 Image 메시지를 퍼블리시하는 퍼블리셔 생성 (큐 사이즈: 10)
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        
        # 0.03초마다 타이머 콜백 실행
        timer_period = 0.03  # 초
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 웹캠 캡처 객체 생성 (예: 인덱스 2의 카메라 사용)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # ROS와 OpenCV 이미지 변환용 CvBridge 객체 생성
        self.br = CvBridge()

    def timer_callback(self):
        """
        타이머 콜백 함수: 웹캠에서 영상을 캡처한 후 ROS Image 메시지로 변환하여 퍼블리시합니다.
        """
        ret, frame = self.cap.read()
        if ret:
            # OpenCV 이미지(frame)가 이미 BGR 형식이므로 encoding을 "bgr8"로 명시하여 ROS 이미지 메시지로 변환
            img_msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(img_msg)
            
        self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

