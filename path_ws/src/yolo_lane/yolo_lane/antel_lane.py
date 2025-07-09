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
        self.slice_step = 10            # Y축 슬라이스 간격(px)
        self.slice_bin = 10             # bin 폭(px)
        self.min_points_per_bin = 1     # 최소 점 개수
        self.approx_epsilon = 5.0       # contour 단순화
        self.num_bottom_slices = 3      # 몇 쌍 벡터로 tangent 평균 낼지
        self.x_jump_threshold = 50      # jump filter는 필요하면 넣기
        self.img_w = 640
        self.img_h = 480

        self.angle_mode = 'vector'      # polyfit X, vector만 사용

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
        approx = cv2.approxPolyDP(contour, self.approx_epsilon, True)
        approx_pts = approx.reshape(-1, 2)

        edge_mask = np.zeros((self.img_h, self.img_w), dtype=np.uint8)
        cv2.polylines(edge_mask, [approx_pts], isClosed=True, color=255, thickness=2)

        contours, _ = cv2.findContours(edge_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 0:
            self.get_logger().info("📭 Contour 없음 → steering=0")
            self.pub.publish(Float32(data=0.0))
            return

        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        contour_pts = contours[0].reshape(-1, 2)

        # ===== 아래(Y 큰) → 위(Y 작은) slicing =====
        y_min, y_max = np.min(contour_pts[:, 1]), np.max(contour_pts[:, 1])
        y_slices = np.arange(y_min, y_max, self.slice_step)[::-1]

        centerline_pts = []
        for y in y_slices:
            bin_mask = (contour_pts[:, 1] >= y - self.slice_bin / 2) & (contour_pts[:, 1] < y + self.slice_bin / 2)
            bin_pts = contour_pts[bin_mask]

            if len(bin_pts) < self.min_points_per_bin:
                continue

            xs = bin_pts[:, 0]
            center_x = (np.min(xs) + np.max(xs)) / 2.0

            centerline_pts.append([center_x, y])

        if len(centerline_pts) < self.num_bottom_slices + 1:
            self.get_logger().info("📭 중심점 부족 → steering=0")
            self.pub.publish(Float32(data=0.0))
            return

        centerline_pts = np.array(centerline_pts)

        # ===== 하단 인접점 벡터 tangent 평균 =====
        angle = self._calc_angle_vector(centerline_pts)
        if angle is None:
            self.get_logger().info("📭 각도 계산 실패 → steering=0")
            self.pub.publish(Float32(data=0.0))
            return

        self.pub.publish(Float32(data=angle))
        self.get_logger().info(f"✅ Steering angle (vector): {angle:.2f}°")

        # ===== 시각화 =====
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

    def _calc_angle_vector(self, pts):
        if len(pts) < 2:
            return None

        # 아래→위 순서 확인 (이미 slicing에서 역순)
        bottom_pts = pts[:self.num_bottom_slices + 1]

        angles = []
        for i in range(len(bottom_pts) - 1):
            dx = bottom_pts[i + 1][0] - bottom_pts[i][0]
            dy = bottom_pts[i + 1][1] - bottom_pts[i][1]
            if abs(dy) < 1e-6:
                continue

            angle_rad = np.arctan2(dx, dy)
            angle_deg = np.rad2deg(angle_rad)

            # 좌측 0도 ~ 우측 180도
            angle_deg = (angle_deg + 360) % 360
            angle_deg = angle_deg if angle_deg <= 180 else 360 - angle_deg

            angles.append(angle_deg)

        if len(angles) == 0:
            return None

        return np.mean(angles)

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

