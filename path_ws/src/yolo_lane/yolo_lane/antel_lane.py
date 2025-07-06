#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import PolygonStamped
import numpy as np
import cv2


class SimpleLaneAngleEstimator(Node):
    def __init__(self):
        super().__init__('simple_lane_angle_node')

        # ===== 하이퍼파라미터 =====
        self.slice_step = 10         # y 슬라이스 간격(px)
        self.slice_bin = 10          # bin 폭(px)
        self.min_points_per_bin = 1  # 슬라이스 최소 점 개수 (vertex sparse할 때는 1로)
        self.polyfit_degree = 3      # 다항식 차수
        self.y_for_angle = 300       # steering angle 계산할 y위치(px)

        self.approx_epsilon = 5.0    # contour 단순화 허용 오차(px)

        self.img_w = 640
        self.img_h = 480

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.sub = self.create_subscription(
            PolygonStamped,
            '/yolo_polygon',
            self.callback,
            qos
        )
        self.pub = self.create_publisher(Float32, '/lane_steering_angle', qos)

        self.get_logger().info("✅ SimpleLaneAngleEstimator: contour 기반 slicing 버전 시작!")

    def callback(self, msg):
        if len(msg.polygon.points) < 3:
            self.get_logger().info("📭 Polygon point 부족")
            return

        # ===== Polygon → NumPy =====
        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.int32)

        # ===== 외곽선 단순화 =====
        contour = pts.reshape((-1, 1, 2))  # (N,1,2)
        approx = cv2.approxPolyDP(contour, self.approx_epsilon, True)
        approx_pts = approx.reshape(-1, 2)

        # ===== Edge mask =====
        edge_mask = np.zeros((self.img_h, self.img_w), dtype=np.uint8)
        cv2.polylines(edge_mask, [approx_pts], isClosed=True, color=255, thickness=2)

        # ===== Contour point 추출 =====
        contours, _ = cv2.findContours(edge_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 0:
            self.get_logger().info("📭 Contour point 없음")
            return
        contour_pts = contours[0].reshape(-1, 2)  # (N,2)

        # ===== Edge 시각화 =====
        cv2.imshow("Polygon Edge (approx)", edge_mask)

        # ===== slicing =====
        y_min, y_max = np.min(contour_pts[:, 1]), np.max(contour_pts[:, 1])
        y_slices = np.arange(y_min, y_max, self.slice_step)

        centerline_pts = []
        for y in y_slices:
            bin_mask = (contour_pts[:, 1] >= y - self.slice_bin / 2) & (contour_pts[:, 1] < y + self.slice_bin / 2)
            bin_pts = contour_pts[bin_mask]

            if len(bin_pts) < self.min_points_per_bin:
                continue

            xs = bin_pts[:, 0]
            left_x = np.min(xs)
            right_x = np.max(xs)
            center_x = (left_x + right_x) / 2.0
            centerline_pts.append([center_x, y])

        if len(centerline_pts) < 5:
            self.get_logger().info("📭 slicing 결과 유효 중점 부족")
            return

        centerline_pts = np.array(centerline_pts)

        # ===== 다항식 피팅 =====
        fit = np.polyfit(centerline_pts[:, 1], centerline_pts[:, 0], self.polyfit_degree)
        poly = np.poly1d(fit)

        # ===== 시각화 =====
        y_fit = np.linspace(y_min, y_max, 100)
        x_fit = poly(y_fit)
        edge_color = cv2.cvtColor(edge_mask, cv2.COLOR_GRAY2BGR)
        for x, y in centerline_pts:
            cv2.circle(edge_color, (int(x), int(y)), 2, (0, 255, 0), -1)  # slicing 중점 (초록)
        for i in range(len(y_fit) - 1):
            pt1 = (int(x_fit[i]), int(y_fit[i]))
            pt2 = (int(x_fit[i + 1]), int(y_fit[i + 1]))
            cv2.line(edge_color, pt1, pt2, (0, 0, 255), 1)  # polyfit 곡선 (빨강)

        cv2.imshow("Centerline Polyfit (contour)", edge_color)
        cv2.waitKey(1)

        # ===== steering angle =====
        dy = 1.0
        y0 = self.y_for_angle
        dx = (poly(y0 + dy) - poly(y0 - dy)) / (2 * dy)
        angle_rad = np.arctan(dx)
        angle_deg = np.rad2deg(angle_rad)
        steering_angle = 90.0 + angle_deg  # 90° = 직진

        self.pub.publish(Float32(data=steering_angle))
        self.get_logger().info(f"✅ Steering angle: {steering_angle:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleLaneAngleEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

