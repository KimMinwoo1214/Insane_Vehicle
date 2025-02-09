#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import torch
import torchvision.models as models
import torch.nn as nn
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from torchvision import transforms
from PIL import Image

class ROICrop:

    def __call__(self, img):
        width, height = img.size

        if width != 640 or height != 360:
            raise ValueError("Expected image size of 640x360")

        y_max = height // 2
        return img.crop((0, 0, width, y_max))

class EmphasizeRed:

    def __call__(self, img):

        img = np.array(img)

        img[:, :, 0] = img[:, :, 0] * 1.0
        img[:, :, 1] = img[:, :, 1] * 1.0
        img[:, :, 2] = img[:, :, 2] * 1.0

        img = Image.fromarray(np.uint8(img))
        return img

class TrafficDetection:
    def __init__(self):

        self.bridge = CvBridge()
        self.result_publish = rospy.Publisher("/traffic_res", String, queue_size=1)
        self.image_sub = rospy.Subscriber("/image_jpeg/camera01/compressed", CompressedImage, self.callback)


        self.signal_dict = {0: "none", 1: "red"}


        self.frame_counter = 0
        self.frame_interval = 4

        self.result = String()


        self.model = models.alexnet(weights=None)
        self.model.classifier[6] = nn.Linear(self.model.classifier[6].in_features, 2)

        # 저장된 학습된 모델 가중치 로드 (weights_only=True 사용)
        state_dict = torch.load('/home/mobilion/catkin_ws/src/aMAP/src/models/traffic_alexnet_roi_final.pt', map_location='cuda', weights_only=True)
        self.model.load_state_dict(state_dict)

        self.model.eval()
        self.model.to('cuda')

        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            ROICrop(),
            EmphasizeRed(), # 이미지 색상 강조 함수
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
        ])

        rospy.loginfo("TrafficDetection node initialized with model and transform settings.")

    def callback(self, msg):
        rospy.loginfo("Received image")


        self.frame_counter += 1
        if self.frame_counter % self.frame_interval == 0:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

            if img_rgb is not None:

                input_tensor = self.transform(img_rgb)
                input_batch = input_tensor.unsqueeze(0).to('cuda')

                with torch.no_grad():
                    outputs = self.model(input_batch)
                    prediction = torch.argmax(outputs, dim=1).item()


                self.result.data = self.signal_dict[prediction]
                rospy.loginfo(f"Published signal: {self.result.data}")
                self.signal_publish(self.result)
            else:
                rospy.logwarn("Received empty image.")

    def signal_publish(self, result):

        self.result_publish.publish(result)


if __name__ == '__main__':
    rospy.init_node('traffic_publisher', anonymous=True)
    td = TrafficDetection()
    rospy.spin()