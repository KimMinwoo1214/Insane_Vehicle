#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import PolygonStamped
import numpy as np
import cv2
from collections import defaultdict

class HybridMidlineAngleEstimator(Node):
    def __init__(self):
        super().__init__('hybrid_midline_angle_node')

        self.img_w = 640
        self.img_h = 480
        self.roi_ymin = 300
        self.roi_ymax = 400
        self.reference_point = np.array([320.0, 480.0])
        self.num_bottom_slices = 8
        self.alpha = 0.005  # 중심 유지 비중
        self.visualize = True

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

        self.get_logger().info("✅ Hybrid Midline Angle 노드 시작됨")

    def callback(self, msg):
        if len(msg.polygon.points) < 3:
            self.pub.publish(Float32(data=0.0))
            return

        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.int32)

        # === contour 추출 ===
        mask = np.zeros((self.img_h, self.img_w), dtype=np.uint8)
        cv2.polylines(mask, [pts.reshape(-1, 1, 2)], isClosed=True, color=255, thickness=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 0:
            self.pub.publish(Float32(data=0.0))
            return
        contour_pts = contours[0].reshape(-1, 2)

        # === ROI 필터링 ===
        roi_pts = contour_pts[(contour_pts[:, 1] >= self.roi_ymin) & (contour_pts[:, 1] <= self.roi_ymax)]

        # === y값 기준으로 그룹핑 → 각 y에서 x 평균값 추출 ===
        y_to_xs = defaultdict(list)
        for x, y in roi_pts:
            y_to_xs[int(y)].append(x)

        mid_pts = []
        for y in sorted(y_to_xs.keys()):
            xs = y_to_xs[y]
            x_mean = np.mean(xs)
            mid_pts.append([x_mean, y])

        mid_pts = np.array(mid_pts, dtype=np.float32)
        if len(mid_pts) < self.num_bottom_slices + 1:
            self.pub.publish(Float32(data=0.0))
            return

        # === 중심 기준 각도 ===
        angles_center = []
        for pt in mid_pts:
            dx = pt[0] - self.reference_point[0]
            dy = self.reference_point[1] - pt[1]
            if dy == 0:
                continue
            angle_rad = np.arctan2(dx, dy)
            angles_center.append(90.0 + np.rad2deg(angle_rad))
        angle_centered = np.mean(angles_center) if angles_center else 90.0

        # === 하단 tangent 평균 각도 ===
        bottom_pts = mid_pts[:self.num_bottom_slices + 1]
        angles_tangent = []
        for i in range(len(bottom_pts) - 1):
            dx = bottom_pts[i + 1][0] - bottom_pts[i][0]
            dy = bottom_pts[i + 1][1] - bottom_pts[i][1]
            if abs(dy) < 1e-6:
                continue
            angle_rad = np.arctan2(dx, dy)
            angle_deg = np.rad2deg(angle_rad)
            angle_deg = (angle_deg + 360) % 360
            angle_deg = angle_deg if angle_deg <= 180 else 360 - angle_deg
            angles_tangent.append(angle_deg)
        angle_tangent = np.mean(angles_tangent) if angles_tangent else 90.0

        # === 조합 ===
        final_angle = (1 - self.alpha) * angle_tangent + self.alpha * angle_centered
        self.pub.publish(Float32(data=final_angle))
        self.get_logger().info(f"✅ Hybrid: tangent={angle_tangent:.2f}, center={angle_centered:.2f} → final={final_angle:.2f}°")

        # === 시각화 ===
        if self.visualize:
            vis_img = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
            for pt in mid_pts:
                cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 2, (0, 255, 0), -1)
            for i in range(len(bottom_pts) - 1):
                pt1 = tuple(bottom_pts[i].astype(int))
                pt2 = tuple(bottom_pts[i + 1].astype(int))
                cv2.line(vis_img, pt1, pt2, (0, 255, 255), 1)
            cv2.circle(vis_img, tuple(self.reference_point.astype(int)), 4, (255, 255, 255), -1)
            cv2.putText(vis_img, f"{final_angle:.2f} deg", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow("Midline-Based Steering", vis_img)
            cv2.waitKey(1)

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = HybridMidlineAngleEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
