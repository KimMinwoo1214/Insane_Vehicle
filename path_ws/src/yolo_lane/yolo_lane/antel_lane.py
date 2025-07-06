#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
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
        self.img_w = 640
        self.img_h = 480

        self.angle_mode = 'vector'    #'polyfit' 또는 'vector'
        self.y_target_ratio = 0.9       # 상단 목표 지점 (ex: 0.2 = 위에서 20%)

        self.create_subscription(
            PolygonStamped,
            '/yolo_polygon',
            self.callback,
            10
        )
        self.pub = self.create_publisher(Float32, '/lane_steering_angle', 10)

        self.get_logger().info(f"✅ SimpleLaneAngleEstimator 시작 (mode: {self.angle_mode})")

    def callback(self, msg):
        try:
            if len(msg.polygon.points) < 3:
                self.get_logger().info("📭 Polygon point 부족")
                return

            pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.int32)
            y_min, y_max = np.min(pts[:, 1]), np.max(pts[:, 1])
            y_slices = np.arange(y_min, y_max, self.slice_step)

            centerline_pts = []
            prev_x = None
            for y in y_slices:
                bin_mask = (pts[:, 1] >= y - self.slice_bin / 2) & (pts[:, 1] < y + self.slice_bin / 2)
                bin_pts = pts[bin_mask]

                if len(bin_pts) < self.min_points_per_bin:
                    continue

                xs = bin_pts[:, 0]
                center_x = (np.min(xs) + np.max(xs)) / 2.0

                if prev_x is not None and abs(center_x - prev_x) > 50:
                    continue

                centerline_pts.append([center_x, y])
                prev_x = center_x

            if len(centerline_pts) < 5:
                self.get_logger().info("📭 유효 중심점 부족")
                return

            centerline_pts = np.array(centerline_pts)

            # ===== 각도 계산 =====
            if self.angle_mode == 'polyfit':
                angle = self._calc_angle_polyfit(centerline_pts)
            elif self.angle_mode == 'vector':
                angle = self._calc_angle_vector(centerline_pts)
            else:
                self.get_logger().warn(f"⚠️ 잘못된 angle_mode: {self.angle_mode}")
                return

            if angle is None:
                self.get_logger().info("📭 조향각 계산 실패")
                return

            self.pub.publish(Float32(data=angle))
            self.get_logger().info(f"✅ Steering angle ({self.angle_mode}): {angle:.2f}°")

            # ===== 시각화 =====
            vis = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
            for x, y in centerline_pts:
                cv2.circle(vis, (int(x), int(y)), 2, (0, 255, 0), -1)
            cv2.putText(vis, f"{angle:.2f} deg", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.imshow("Centerline", vis)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"[Callback Error] {e}")

    def _calc_angle_polyfit(self, pts):
        try:
            fit = np.polyfit(pts[:, 1], pts[:, 0], self.polyfit_degree)
            poly = np.poly1d(fit)

            y_target = np.clip(int(self.img_h * (1.0 - self.y_target_ratio)),
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
            self.get_logger().warn(f"[polyfit 실패] {e}")
            return None

    def _calc_angle_vector(self, pts):
        bottom = pts[np.argmax(pts[:, 1])]
        y_thresh = self.img_h * (1.0 - self.y_target_ratio)
        target_candidates = pts[pts[:, 1] < y_thresh]

        if len(target_candidates) == 0:
            return None

        angles = []
        for pt in target_candidates:
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
