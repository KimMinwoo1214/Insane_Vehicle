#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import PolygonStamped
import numpy as np
import cv2

class HybridTangentLaneAngleEstimator(Node):
    def __init__(self):
        super().__init__('hybrid_tangent_lane_angle_node')

        # ===== 하이퍼파라미터 =====
        self.img_w = 640
        self.img_h = 480
        self.roi_ymin = 300
        self.roi_ymax = 400
        self.slice_step = 10
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

        self.get_logger().info("✅ Hybrid (tangent+center) 조향각 노드 시작됨")

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

        if len(filtered_pts) < self.num_bottom_slices + 1:
            self.pub.publish(Float32(data=0.0))
            return

        # === 중심 기준 기울기 ===
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

        # === 하단 연속 tangent 평균 ===
        bottom_pts = filtered_pts[:self.num_bottom_slices + 1]
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
        angle_tangent = np.mean(angles_tangent) if len(angles_tangent) > 0 else 90.0

        # === 하이브리드 조합 ===
        final_angle = (1 - self.alpha) * angle_tangent + self.alpha * angle_centered
        self.pub.publish(Float32(data=final_angle))
        self.get_logger().info(f"✅ Hybrid angle: tangent={angle_tangent:.2f}°, center={angle_centered:.2f}° → final={final_angle:.2f}°")

        # === 시각화 ===
        if self.visualize:
            vis_img = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
            for pt in filtered_pts:
                cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 2, (0, 255, 0), -1)
            for i in range(len(bottom_pts) - 1):
                pt1 = tuple(bottom_pts[i].astype(int))
                pt2 = tuple(bottom_pts[i + 1].astype(int))
                cv2.line(vis_img, pt1, pt2, (0, 255, 255), 1)
            cv2.circle(vis_img, tuple(self.reference_point.astype(int)), 4, (255, 255, 255), -1)
            cv2.putText(vis_img, f"{final_angle:.2f} deg", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow("Hybrid (tangent + center)", vis_img)
            cv2.waitKey(1)

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = HybridTangentLaneAngleEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
