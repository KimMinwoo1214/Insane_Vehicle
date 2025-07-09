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
        self.slice_step = 10
        self.slice_bin = 10
        self.min_points_per_bin = 2
        self.num_bottom_slices = 3   # 하단 몇 슬라이스로 tangent 평균낼지
        self.approx_epsilon = 10.0

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

        self.get_logger().info("✅ SimpleLaneAngleEstimator 시작됨 (하단 N슬라이스 tangent 평균 모드)")

    def callback(self, msg):
        if len(msg.polygon.points) < 3:
            self.get_logger().info("📭 Polygon point 부족")
            return

        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.int32)

        # ===== 다각형 단순화 및 edge mask 생성 =====
        contour = pts.reshape((-1, 1, 2))
        approx = cv2.approxPolyDP(contour, self.approx_epsilon, True)
        approx_pts = approx.reshape(-1, 2)

        edge_mask = np.zeros((self.img_h, self.img_w), dtype=np.uint8)
        cv2.polylines(edge_mask, [approx_pts], isClosed=True, color=255, thickness=2)

        kernel = np.ones((3, 3), np.uint8)
        edge_mask = cv2.morphologyEx(edge_mask, cv2.MORPH_OPEN, kernel)

        # ===== 외곽선 추출 =====
        contours, _ = cv2.findContours(edge_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 0:
            self.get_logger().info("📭 Contour 없음")
            return

        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        contour_pts = contours[0].reshape(-1, 2)

        # ===== 슬라이싱하여 중심점 추출 =====
        y_min, y_max = np.min(contour_pts[:, 1]), np.max(contour_pts[:, 1])
        y_slices = np.arange(y_min, y_max, self.slice_step)

        centerline_pts = []
        prev_x = None
        for y in y_slices:
            bin_mask = (contour_pts[:, 1] >= y - self.slice_bin / 2) & (contour_pts[:, 1] < y + self.slice_bin / 2)
            bin_pts = contour_pts[bin_mask]

            if len(bin_pts) < self.min_points_per_bin:
                continue

            xs = bin_pts[:, 0]
            center_x = (np.min(xs) + np.max(xs)) / 2.0

            # 연속성 필터
            if prev_x is not None and abs(center_x - prev_x) > 50:
                continue

            centerline_pts.append([center_x, y])
            prev_x = center_x

        if len(centerline_pts) < self.num_bottom_slices + 1:
            self.get_logger().info("📭 중심점 부족")
            return

        centerline_pts = np.array(centerline_pts)

        # ===== 하단 N슬라이스로 tangent 평균 =====
        bottom_pts = centerline_pts[:self.num_bottom_slices + 1]

        angles = []
        for i in range(len(bottom_pts) - 1):
            dx = bottom_pts[i + 1][0] - bottom_pts[i][0]
            dy = bottom_pts[i + 1][1] - bottom_pts[i][1]
            if dy == 0:
                continue
            angle_rad = np.arctan2(dx, dy)
            angles.append(np.rad2deg(angle_rad))

        if len(angles) == 0:
            self.get_logger().info("📭 하단 각도 계산 실패")
            return

        steering_angle = 90.0 + np.mean(angles)
        self.pub.publish(Float32(data=steering_angle))
        self.get_logger().info(f"✅ Steering angle: {steering_angle:.2f}° (하단 N슬라이스 tangent 평균)")

        # ===== 시각화 (붙여준 버전 스타일) =====
        edge_color = cv2.cvtColor(edge_mask, cv2.COLOR_GRAY2BGR)
        for x, y in centerline_pts:
            cv2.circle(edge_color, (int(x), int(y)), 2, (0, 255, 0), -1)
        cv2.putText(edge_color, f"{steering_angle:.2f} deg", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.imshow("Lane Centerline + Angle", edge_color)
        cv2.waitKey(1)

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

