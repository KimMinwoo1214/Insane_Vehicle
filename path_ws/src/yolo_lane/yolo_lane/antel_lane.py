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
        self.min_points_per_bin = 1
        self.polyfit_degree = 2
        self.angle_mode = 'vector'  # 또는 'polyfit'

        # ROI 범위 (픽셀 단위, 퍼셉션 노드와 동일하게 맞춤)
        self.img_w = 640
        self.img_h = 480
        self.roi_ymin = 0
        self.roi_ymax = 480

        # 하단 ROI 중 몇 퍼센트를 사용할지 (예: 하단 30% → 0.7)
        self.lower_ratio_for_angle = 0.85

        # 필터링 기준
        self.min_lane_width = 30
        self.max_lane_width = 100
        self.max_dx_between_slices = 30

        self.approx_epsilon = 5.0

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

        self.get_logger().info(f"✅ SimpleLaneAngleEstimator 시작! (mode: {self.angle_mode})")

    def callback(self, msg):
        if len(msg.polygon.points) < 3:
            self.get_logger().info("📭 Polygon point 부족 → steering=0")
            self.pub.publish(Float32(data=0.0))
            return

        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.int32)
        contour = pts.reshape((-1, 1, 2))

        edge_mask = np.zeros((self.img_h, self.img_w), dtype=np.uint8)
        cv2.polylines(edge_mask, [contour], isClosed=True, color=255, thickness=2)

        contours, _ = cv2.findContours(edge_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 0:
            self.get_logger().info("📭 Contour point 없음 → steering=0")
            self.pub.publish(Float32(data=0.0))
            return
        contour_pts = contours[0].reshape(-1, 2)

        # ROI 범위에 맞춘 y-slice
        y_min = max(np.min(contour_pts[:, 1]), self.roi_ymin)
        y_max = min(np.max(contour_pts[:, 1]), self.roi_ymax)
        y_slices = np.arange(y_min, y_max, self.slice_step)

        centerline_pts = []
        for y in y_slices:
            bin_mask = (contour_pts[:, 1] >= y - self.slice_bin / 2) & (contour_pts[:, 1] < y + self.slice_bin / 2)
            bin_pts = contour_pts[bin_mask]

            if len(bin_pts) < self.min_points_per_bin:
                continue

            xs = bin_pts[:, 0]
            lane_width = np.max(xs) - np.min(xs)
            if not (self.min_lane_width <= lane_width <= self.max_lane_width):
                continue

            center_x = (np.min(xs) + np.max(xs)) / 2.0
            centerline_pts.append([center_x, y])

        # 연속성 필터
        filtered_pts = []
        for i, pt in enumerate(centerline_pts):
            if i == 0:
                filtered_pts.append(pt)
                continue
            dx = abs(pt[0] - filtered_pts[-1][0])
            if dx <= self.max_dx_between_slices:
                filtered_pts.append(pt)

        if len(filtered_pts) < 5:
            self.get_logger().info("📭 유효 중점 부족 → steering=0")
            self.pub.publish(Float32(data=0.0))
            return

        centerline_pts = np.array(filtered_pts)

        # 조향각 계산
        if self.angle_mode == 'polyfit':
            angle = self._calc_angle_polyfit(centerline_pts)
        elif self.angle_mode == 'vector':
            angle = self._calc_angle_vector(centerline_pts)
        else:
            self.get_logger().warn(f"⚠️ 지원하지 않는 mode: {self.angle_mode}")
            self.pub.publish(Float32(data=0.0))
            return

        if angle is None:
            self.get_logger().info("📭 조향각 계산 실패 → steering=0")
            self.pub.publish(Float32(data=0.0))
            return

        self.pub.publish(Float32(data=angle))
        self.get_logger().info(f"✅ Steering angle ({self.angle_mode}): {angle:.2f}°")

        # 시각화
        edge_color = cv2.cvtColor(edge_mask, cv2.COLOR_GRAY2BGR)
        for x, y in centerline_pts:
            cv2.circle(edge_color, (int(x), int(y)), 2, (0, 255, 0), -1)
        for i in range(len(centerline_pts) - 1):
            pt1 = (int(centerline_pts[i][0]), int(centerline_pts[i][1]))
            pt2 = (int(centerline_pts[i + 1][0]), int(centerline_pts[i + 1][1]))
            cv2.line(edge_color, pt1, pt2, (0, 255, 255), 1)

        cv2.putText(edge_color, f"{angle:.2f} deg", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.imshow("Centerline + Angle", edge_color)
        cv2.waitKey(1)

    def _calc_angle_polyfit(self, pts):
        try:
            fit = np.polyfit(pts[:, 1], pts[:, 0], self.polyfit_degree)
            poly = np.poly1d(fit)
            y0 = np.clip(360, np.min(pts[:, 1]) + 5, np.max(pts[:, 1]) - 5)
            dy = 1.0
            dx = (poly(y0 + dy) - poly(y0 - dy)) / (2 * dy)
            angle_rad = np.arctan(dx)
            return 90.0 + np.rad2deg(angle_rad)
        except Exception as e:
            self.get_logger().warn(f"[polyfit 실패] {e}")
            return None

    def _calc_angle_vector(self, pts):
        bottom = pts[np.argmax(pts[:, 1])]
        y_thresh = self.img_h * self.lower_ratio_for_angle
        lower_pts = pts[pts[:, 1] > y_thresh]

        if len(lower_pts) < 1:
            return None

        angles = []
        for pt in lower_pts:
            dx = pt[0] - bottom[0]
            dy = bottom[1] - pt[1]
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
