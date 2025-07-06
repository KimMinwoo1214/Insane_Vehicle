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
        self.polyfit_degree = 2
        self.approx_epsilon = 10.0
        self.y_target_ratio = 0.8         # 상단으로부터의 위치 (0.8 = 80% = 밑 1/5 지점)

        self.angle_mode = 'vector'        # 'polyfit' or 'vector'

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

        self.get_logger().info(f"✅ LaneAngleEstimator 시작 (mode: {self.angle_mode})")

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

        # ===== 외곽선 추출: 면적 큰 것만 사용 =====
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

            if prev_x is not None and abs(center_x - prev_x) > 50:
                continue

            centerline_pts.append([center_x, y])
            prev_x = center_x

        if len(centerline_pts) < 5:
            self.get_logger().info("📭 중심점 부족")
            return

        centerline_pts = np.array(centerline_pts)

        # ===== 조향각 계산 =====
        if self.angle_mode == 'polyfit':
            angle = self._calc_angle_polyfit(centerline_pts)
        elif self.angle_mode == 'vector':
            angle = self._calc_angle_vector(centerline_pts)
        else:
            self.get_logger().warn(f"⚠️ 지원하지 않는 angle_mode: {self.angle_mode}")
            return

        if angle is None:
            self.get_logger().info("📭 유효한 조향각 계산 실패")
            return

        self.pub.publish(Float32(data=angle))
        self.get_logger().info(f"✅ Steering angle ({self.angle_mode}): {angle:.2f}°")

        # ===== 시각화 =====
        edge_color = cv2.cvtColor(edge_mask, cv2.COLOR_GRAY2BGR)
        for x, y in centerline_pts:
            cv2.circle(edge_color, (int(x), int(y)), 2, (0, 255, 0), -1)
        cv2.putText(edge_color, f"{angle:.2f} deg", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.imshow("Lane Centerline + Angle", edge_color)
        cv2.waitKey(1)

    def _calc_angle_polyfit(self, pts):
        try:
            fit = np.polyfit(pts[:, 1], pts[:, 0], self.polyfit_degree)
            poly = np.poly1d(fit)

            y_target = np.clip(int(self.img_h * self.y_target_ratio),
                               np.min(pts[:, 1]) + 5, np.max(pts[:, 1]) - 5)

            angles = []
            for offset in range(-4, 5):
                y = y_target + offset
                dx = (poly(y + 1) - poly(y - 1)) / 2.0
                angle_rad = np.arctan(dx)
                angles.append(np.rad2deg(angle_rad))

            if len(angles) == 0:
                return None
            return 90.0 + np.mean(angles)
        except Exception as e:
            self.get_logger().warn(f"polyfit 실패: {e}")
            return None

    def _calc_angle_vector(self, pts):
        bottom = pts[np.argmax(pts[:, 1])]
        upper_thresh = self.img_h * (1.0 - self.y_target_ratio)  # 상단 1/5 정도
        valid = pts[pts[:, 1] < upper_thresh]

        if len(valid) < 2:
            return None

        angles = []
        for pt in valid:
            dx = pt[0] - bottom[0]
            dy = bottom[1] - pt[1]  # y는 아래로 증가하므로
            if dy == 0:
                continue
            angle_rad = np.arctan2(dx, dy)
            angles.append(np.rad2deg(angle_rad))

        if len(angles) == 0:
            return None
        return 90.0 + np.mean(angles)

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
