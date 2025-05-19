import torch
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torchvision.transforms as transforms
import importlib

class UFLDLanePublisher(Node):
    def __init__(self):
        super().__init__('ufld_lane_publisher')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/lane_path', 10)

        # 모델 로드
        cfg = {
            "dataset": "CULane",
            "train_width": 800,
            "train_height": 320,
            "backbone": "18",
            "row_anchor": [i for i in range(160, 320, 10)],
            "col_anchor": [i for i in range(100, 700, 50)],
            "test_model": "checkpoints/culane_res18.pth",
        }
        model_module = importlib.import_module(f"model.model_{cfg['dataset'].lower()}")
        self.net = model_module.get_model(cfg).cuda()
        state_dict = torch.load(cfg["test_model"], map_location="cuda")["model"]
        self.net.load_state_dict(state_dict, strict=False)
        self.net.eval()

        self.img_transforms = transforms.Compose([
            transforms.Resize((cfg["train_height"], cfg["train_width"])),
            transforms.ToTensor(),
            transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
        ])

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img = self.img_transforms(frame).unsqueeze(0).cuda()

        with torch.no_grad():
            pred = self.net(img)

        # 차선 좌표 추출
        coords = self.pred2coords(pred)

        # 중심선 계산
        center_line = self.calculate_center_line(coords)

        # Marker 메시지 생성 및 퍼블리시
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "camera_link"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for pt in center_line:
            p = Point()
            p.x = pt[0]
            p.y = pt[1]
            p.z = 0.0
            marker.points.append(p)

        self.marker_pub.publish(marker)

    def pred2coords(self, pred):
        # 모델 출력에서 차선 좌표 추출
        # pred['loc_row']: (batch_size, num_grid_row, num_cls_row, num_lane_row)
        batch_size, num_grid_row, num_cls_row, num_lane_row = pred['loc_row'].shape
        max_indices_row = pred['loc_row'].argmax(1).cpu()
        valid_row = pred['exist_row'].argmax(1).cpu()

        coords = []
        row_lane_idx = [1, 2]  # 좌우 차선 인덱스

        for i in row_lane_idx:
            tmp = []
            if valid_row[0, :, i].sum() > num_cls_row / 2:
                for k in range(valid_row.shape[1]):
                    if valid_row[0, k, i]:
                        out_tmp = (pred['loc_row'][0, :, k, i].softmax(0) * torch.arange(len(cfg["row_anchor"])).float()).sum().item()
                        tmp.append((int(out_tmp / len(cfg["row_anchor"]) * cfg["train_width"]), int(cfg["row_anchor"][k] * cfg["train_height"])))
                coords.append(tmp)
        return coords

    def calculate_center_line(self, coords):
        # 좌우 차선의 중간점을 계산하여 중심선 생성
        if len(coords) < 2:
            return []
        left_lane = coords[0]
        right_lane = coords[1]
        center_line = []
        for l_pt, r_pt in zip(left_lane, right_lane):
            center_x = (l_pt[0] + r_pt[0]) / 2
            center_y = (l_pt[1] + r_pt[1]) / 2
            center_line.append((center_x, center_y))
        return center_line

def main(args=None):
    rclpy.init(args=args)
    node = UFLDLanePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
