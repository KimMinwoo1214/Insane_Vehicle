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
from polylanenet.model import PolyLaneNet

class PolyLaneNetPublisher(Node):
    def __init__(self):
        super().__init__('polylanenet_publisher')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/lane_path', 10)

        # 모델 로드
        self.model = PolyLaneNet()
        self.model.load_state_dict(torch.load('checkpoints/polylanenet.pth'))
        self.model.cuda()
        self.model.eval()

        self.img_transforms = transforms.Compose([
            transforms.Resize((288, 800)),
            transforms.ToTensor(),
            transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
        ])

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img = self.img_transforms(frame).unsqueeze(0).cuda()

        with torch.no_grad():
            output = self.model(img)

        # 차선 다항식 계수 추출
        lanes = output['lanes']  # 예: [(a0, a1, a2, a3), ...]

        # 좌우 차선 선택
        if len(lanes) < 2:
            return
        left_lane = lanes[0]
        right_lane = lanes[1]

        # 중심선 계산
        y_vals = np.linspace(0, 1, num=50)
        center_line = []
        for y in y_vals:
            x_left = np.polyval(left_lane, y)
            x_right = np.polyval(right_lane, y)
            center_x = (x_left + x_right) / 2
            center_line.append((center_x, y))

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
        marker.color.g = 0.0
        marker.color.b = 1.0

        for pt in center_line:
            p = Point()
            p.x = pt[0]
            p.y = pt[1]
            p.z = 0.0
            marker.points.append(p)

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PolyLaneNetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
