#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import PolygonStamped
import numpy as np
import cv2

class HybridTargetSliceAngleEstimator(Node):
    def __init__(self):
        super().__init__('hybrid_target_slice_angle_node')

        # ===== 하이퍼파라미터 =====
        self.img_w = 640
        self.img_h = 480
        self.roi_ymin = 300
        self.roi_ymax = 400
        self.reference_point = np.array([320.0, 480.0])  # 기준점
        self.alpha = 0.005  # 중심 유지 비중
        self.slice_step = 10  # 10픽셀 간격
        self.visualize = True

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.sub = self.create_subscription(
            PolygonStamped,
            '/yolo_lane_polygon',
            self.callback,
            qos
        )
        self.pub = self.create_publisher(Float32, '/lane_steering_angle', qos)

        self.get_logger().info("✅ HybridTargetSliceAngleEstimator 시작됨")

    def callback(self, msg):
        if len(msg.polygon.points) < 3:
            self.pub.publish(Float32(data=0.0))
            return

        # Polygon → contour mask 생성
        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.int32)
        mask = np.zeros((self.img_h, self.img_w), dtype=np.uint8)
        cv2.polylines(mask, [pts.reshape(-1, 1, 2)], isClosed=True, color=255, thickness=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            self.pub.publish(Float32(data=0.0))
            return

        # 가장 큰 contour 기준
        contour = max(contours, key=cv2.contourArea)
        contour_pts = contour.reshape(-1, 2)

        # === 타겟 y 지점 9개 (roi_ymax부터 하단 방향으로 slice_step 간격) ===
        y_targets = np.arange(self.roi_ymax, self.roi_ymin - 1, -self.slice_step)
        target_pts = []
        for y in y_targets:
            y_mask = np.abs(contour_pts[:, 1] - y) < self.slice_step / 2
            xs = contour_pts[y_mask][:, 0]
            if len(xs) >= 1:
                center_x = np.mean(xs)
                target_pts.append([center_x, y])
        target_pts = np.array(target_pts)

        if len(target_pts) < 2:
            self.pub.publish(Float32(data=0.0))
            return

        # === 중심 기반 보정용 각도 ===
        ref = self.reference_point
        angles_center = []
        for pt in target_pts:
            dx = pt[0] - ref[0]
            dy = ref[1] - pt[1]
            if dy == 0:
                continue
            angle_rad = np.arctan2(dx, dy)
            angles_center.append(90.0 + np.rad2deg(angle_rad))
        angle_centered = np.mean(angles_center) if len(angles_center) > 0 else 90.0

        # === 기준점 기준 벡터 각도 평균 ===
        angles_direct = []
        for pt in target_pts:
            dx = pt[0] - ref[0]
            dy = ref[1] - pt[1]
            if dy == 0:
                continue
            angle_rad = np.arctan2(dx, dy)
            angles_direct.append(90.0 + np.rad2deg(angle_rad))
        angle_direct = np.mean(angles_direct) if len(angles_direct) > 0 else 90.0

        # === 하이브리드 조합 ===
        final_angle = (1 - self.alpha) * angle_direct + self.alpha * angle_centered
        self.pub.publish(Float32(data=final_angle))
        self.get_logger().info(f"✅ Hybrid angle: direct={angle_direct:.2f}°, center={angle_centered:.2f}° → final={final_angle:.2f}°")

        # === 시각화 ===
        if self.visualize:
            vis = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
            for pt in target_pts:
                pt = tuple(np.round(pt).astype(int))
                cv2.circle(vis, pt, 3, (0, 255, 0), -1)
                cv2.line(vis, tuple(self.reference_point.astype(int)), pt, (255, 255, 255), 1)
            cv2.circle(vis, tuple(self.reference_point.astype(int)), 4, (0, 0, 255), -1)
            cv2.putText(vis, f"{final_angle:.2f} deg", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow("Hybrid Target Angle", vis)
            cv2.waitKey(1)

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = HybridTargetSliceAngleEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
