#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import PolygonStamped
import numpy as np
import cv2

class HybridLaneAngleEstimator(Node):
    def __init__(self):
        super().__init__('hybrid_lane_angle_node')

        # ===== 하이퍼파라미터 =====
        self.img_w = 640
        self.img_h = 480
        self.roi_ymin = 300
        self.roi_ymax = 400
        self.slice_step = 10
        self.reference_point = np.array([320.0, 480.0])
        self.alpha = 0.0501  # 중심 유지에 대한 가중치

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

        self.get_logger().info("✅ Hybrid 조향각 노드 시작됨")

    def callback(self, msg):
        if len(msg.polygon.points) < 2:
            self.pub.publish(Float32(data=0.0))
            return

        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.float32)
        roi_pts = pts[(pts[:, 1] >= self.roi_ymin) & (pts[:, 1] <= self.roi_ymax)]
        if roi_pts.shape[0] < 2:
            self.pub.publish(Float32(data=0.0))
            return

        # y 기준 간격 필터링
        sorted_pts = roi_pts[np.argsort(roi_pts[:, 1])]
        filtered_pts = []
        last_y = -1000
        for pt in sorted_pts:
            if abs(pt[1] - last_y) >= self.slice_step:
                filtered_pts.append(pt)
                last_y = pt[1]
        filtered_pts = np.array(filtered_pts)

        if len(filtered_pts) < 2:
            self.pub.publish(Float32(data=0.0))
            return

        # === 중심 기준 기울기 (reference point 기준) ===
        angles_center = []
        ref = self.reference_point
        for pt in filtered_pts:
            dx = pt[0] - ref[0]
            dy = ref[1] - pt[1]
            if dy == 0:
                continue
            angle_rad = np.arctan2(dx, dy)
            angles_center.append(90.0 + np.rad2deg(angle_rad))

        angle_centered = np.mean(angles_center) if len(angles_center) > 0 else 90.0

        # === 최하단 기준 기울기 ===
        bottom = filtered_pts[np.argmax(filtered_pts[:, 1])]
        angles_bottom = []
        for pt in filtered_pts:
            if np.allclose(pt, bottom):
                continue
            dx = pt[0] - bottom[0]
            dy = bottom[1] - pt[1]
            if dy == 0:
                continue
            angle_rad = np.arctan2(dx, dy)
            angles_bottom.append(90.0 + np.rad2deg(angle_rad))

        angle_bottom = np.mean(angles_bottom) if len(angles_bottom) > 0 else 90.0

        # === 하이브리드 조합 ===
        final_angle = (1 - self.alpha) * angle_bottom + self.alpha * angle_centered
        self.pub.publish(Float32(data=final_angle))
        self.get_logger().info(f"✅ Hybrid angle: center={angle_centered:.2f}°, bottom={angle_bottom:.2f}° → final={final_angle:.2f}°")

        # === 시각화 ===
        if self.visualize:
            vis_img = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
            for pt in filtered_pts:
                cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 2, (0, 255, 0), -1)
                cv2.line(vis_img, (int(ref[0]), int(ref[1])), (int(pt[0]), int(pt[1])), (100, 100, 255), 1)
                cv2.line(vis_img, (int(bottom[0]), int(bottom[1])), (int(pt[0]), int(pt[1])), (255, 100, 100), 1)
            cv2.circle(vis_img, (int(ref[0]), int(ref[1])), 4, (255, 255, 255), -1)
            cv2.circle(vis_img, (int(bottom[0]), int(bottom[1])), 4, (100, 255, 255), -1)
            cv2.putText(vis_img, f"{final_angle:.2f} deg", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow("Hybrid Steering Angle", vis_img)
            cv2.waitKey(1)

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = HybridLaneAngleEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
