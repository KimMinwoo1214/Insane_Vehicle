import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import torch
import argparse
import os
import sys
import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Float64
from std_msgs.msg import Float32MultiArray
sys.path.append(os.path.join(os.path.dirname(__file__), "../"))
from utils.config import Config
import math

MAX_PIXEL = 420
MIN_PIXEL = 520


class UFLDv2(Node):
    def __init__(self, engine_path, config_path, ori_size):
        super().__init__('ufldv2_node')  # Initialize ROS Node

        # Initialize CvBridge and publisher
        self.bridge = CvBridge()
        self.lane_angle = self.create_publisher(Float64, 'lane_steering_angle', 10)
        self.lane_show = self.create_publisher(Image, 'lane_show', 10)
        self.create_subscription(Image, "/video_frames", self.image_callback, 10)


        self.logger = trt.Logger(trt.Logger.ERROR)
        with open(engine_path, "rb") as f, trt.Runtime(self.logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.trt_context = self.engine.create_execution_context()

        self.inputs = []
        self.outputs = []
        self.allocations = []
        for i in range(self.engine.num_io_tensors):
            is_input = self.engine.get_tensor_mode(self.engine.get_tensor_name(i)) == trt.TensorIOMode.INPUT
            name = self.engine.get_tensor_name(i)
            dtype = self.engine.get_tensor_dtype(name)
            shape = self.engine.get_tensor_shape(name)
            if is_input:
                self.batch_size = shape[0]
            size = np.dtype(trt.nptype(dtype)).itemsize
            for s in shape:
                size *= s
            allocation = cuda.mem_alloc(size)
            binding = {
                'index': i,
                'name': name,
                'dtype': np.dtype(trt.nptype(dtype)),
                'shape': list(shape),
                'allocation': allocation,
            }
            self.allocations.append(allocation)
            if is_input:
                self.inputs.append(binding)
            else:
                self.outputs.append(binding)

        cfg = Config.fromfile(config_path)
        self.ori_img_w, self.ori_img_h = ori_size
        self.cut_height = int(cfg.train_height * (1 - cfg.crop_ratio))
        self.input_width = cfg.train_width
        self.input_height = cfg.train_height
        self.num_row = cfg.num_row
        self.num_col = cfg.num_col
        self.row_anchor = np.linspace(0.42, 1, self.num_row)
        self.col_anchor = np.linspace(0, 1, self.num_col)

    def image_callback(self, msg):
        """ 카메라 이미지 수신 후 lane detection 처리 """
        self.get_logger().info('Receiving video frame')

        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환 (BGR 형식)
            current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            current_frame = cv2.resize(current_frame, (640, 480))

            # Convert to HSV color space
            hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

            # Define color ranges for white and yellow

            # current_frame = cv2.GaussianBlur(current_frame, (5, 5), 0)
            # 수신 프레임 저장
            self.img_bgr = current_frame.copy()

            # Forward pass through the network
            self.forward(current_frame)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def pred2coords(self, pred):
        batch_size, num_grid_row, num_cls_row, num_lane_row = pred['loc_row'].shape
        batch_size, num_grid_col, num_cls_col, num_lane_col = pred['loc_col'].shape

        max_indices_row = pred['loc_row'].argmax(1)
        # n , num_cls, num_lanes
        valid_row = pred['exist_row'].argmax(1)
        # n, num_cls, num_lanes

        max_indices_col = pred['loc_col'].argmax(1)
        # n , num_cls, num_lanes
        valid_col = pred['exist_col'].argmax(1)
        # n, num_cls, num_lanes

        pred['loc_row'] = pred['loc_row']
        pred['loc_col'] = pred['loc_col']

        coords = []


        row_lane_idx = [3, 4, 5, 6]
        col_lane_idx = [2, 7]

        for i in row_lane_idx:
            tmp = []
            if valid_row[0, :, i].sum() > num_cls_row / 2:
                for k in range(valid_row.shape[1]):
                    if valid_row[0, k, i]:
                        all_ind = torch.tensor(list(range(max(0, max_indices_row[0, k, i] - self.input_width),
                                                          min(num_grid_row - 1,
                                                              max_indices_row[0, k, i] + self.input_width) + 1)))

                        out_tmp = (pred['loc_row'][0, all_ind, k, i].softmax(0) * all_ind.float()).sum() + 0.5
                        out_tmp = out_tmp / (num_grid_row - 1) * self.ori_img_w
                        tmp.append((int(out_tmp), int(self.row_anchor[k] * self.ori_img_h)))
                coords.append(tmp)

        for i in col_lane_idx:
            tmp = []
            if valid_col[0, :, i].sum() > num_cls_col / 4:
                for k in range(valid_col.shape[1]):
                    if valid_col[0, k, i]:
                        all_ind = torch.tensor(list(range(max(0, max_indices_col[0, k, i] - self.input_width),
                                                          min(num_grid_col - 1,
                                                              max_indices_col[0, k, i] + self.input_width) + 1)))
                        out_tmp = (pred['loc_col'][0, all_ind, k, i].softmax(0) * all_ind.float()).sum() + 0.5
                        out_tmp = out_tmp / (num_grid_col - 1) * self.ori_img_h
                        tmp.append((int(self.col_anchor[k] * self.ori_img_w), int(out_tmp)))
                coords.append(tmp)
        return coords

    def forward(self, img):
        im0 = img.copy()
        img = img[self.cut_height:, :, :]
        img = cv2.resize(img, (self.input_width, self.input_height), cv2.INTER_CUBIC)
        img = img.astype(np.float16) / 255.0
        img = np.transpose(np.float16(img[:, :, :, np.newaxis]), (3, 2, 0, 1))
        img = np.ascontiguousarray(img)
        cuda.memcpy_htod(self.inputs[0]['allocation'], img)
        self.trt_context.execute_v2(self.allocations)
        preds = {}
        for out in self.outputs:
            output = np.zeros(out['shape'], out['dtype'])
            cuda.memcpy_dtoh(output, out['allocation'])
            preds[out['name']] = torch.tensor(output)
        coords = self.pred2coords(preds)
        return_coords = []
        if len(coords) == 2:
            target = min(len(coords[0]), len(coords[1]))
            cord1 = coords[0][target]
            cord2 = coords[1][target]
            print(math.dist(cord1 , cord2))
            if math.dist(cord1 , cord2) > 140:
                min = (cord1 + cord2) / 2

        if len(coords) == 1 or len(return_coords) == 1:
            if len(return_coords) != 1:
                return_coords = coords[0]
            tangent = []
            for i in range(1, len(return_coords)):
                if return_coords[i][1] >= MAX_PIXEL and return_coords[i][1] <= MIN_PIXEL:
                    tangent.append(math.atan2((return_coords[i][1] - return_coords[i - 1][1]), (return_coords[i][0] - return_coords[i - 1][0])))

            average = 0
            for i in tangent:
                average += i
            if len(tangent) != 0:
                average = average / len(tangent)
                deg = 90 - average
                deg = np.clip(deg, 67.5, 112.5)
            elif len(tangent) ==0:
                deg = 0.0
            msg = Float64()
            msg.data = float(deg)
            self.lane_angle.publish(msg)

        for lane in coords:
            for coord in lane:
                cv2.circle(im0, coord, 2, (0, 255, 0), -1)

        img_msg = self.bridge.cv2_to_imgmsg(im0, encoding="bgr8")
        self.lane_show.publish(img_msg)

def main():
    config_path = '/home/parkm04/Desktop/Ultra-Fast-Lane-Detection-v2/configs/curvelanes_res18.py'
    engine_path = '/home/parkm04/Desktop/Ultra-Fast-Lane-Detection-v2/0629lane.engine'
    ori_size = (640, 480)

    try:
        rclpy.init()
        detector = UFLDv2(engine_path, config_path, ori_size)
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        if 'detector' in locals():
            detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()