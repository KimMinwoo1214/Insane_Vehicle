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
        self.img_w = 640
        self.img_h = 480
        self.roi_ymin = 240  # ROI 상단 (픽셀)
        self.roi_ymax = 360  # ROI 하단 (픽셀)
        self.reference_point = np.array([320.0, 480.0])  # 기준점: 이미지 하단 중앙
        self.slice_step = 15  # 후보점 간 y간격
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
        self.get_logger().info("✅ Center 기준 + ROI + 간격 기반 조향각 노드 시작!")

    def callback(self, msg):
        if len(msg.polygon.points) < 2:
            self.pub.publish(Float32(data=0.0))
            return

        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.float32)

        # === ROI 범위 필터링 ===
        roi_mask = (pts[:, 1] >= self.roi_ymin) & (pts[:, 1] <= self.roi_ymax)
        roi_pts = pts[roi_mask]
        if roi_pts.shape[0] < 2:
            self.get_logger().info("📭 ROI 점 부족 → steering=0")
            self.pub.publish(Float32(data=0.0))
            return

        # === 후보점 간격 필터링 (y 값 기준) ===
        sorted_pts = roi_pts[np.argsort(roi_pts[:, 1])]  # y기준 오름차순 정렬
        filtered_pts = []
        last_y = -1000
        for pt in sorted_pts:
            if abs(pt[1] - last_y) >= self.slice_step:
                filtered_pts.append(pt)
                last_y = pt[1]
        filtered_pts = np.array(filtered_pts)

        if len(filtered_pts) < 2:
            self.get_logger().info("📭 간격 필터 후 점 부족 → steering=0")
            self.pub.publish(Float32(data=0.0))
            return

        # === 기울기 계산 ===
        ref = self.reference_point
        angles = []
        for pt in filtered_pts:
            dx = pt[0] - ref[0]
            dy = ref[1] - pt[1]
            if dy == 0:
                continue
            angle_rad = np.arctan2(dx, dy)
            angle_deg = 90.0 + np.rad2deg(angle_rad)
            angles.append(angle_deg)

        if len(angles) == 0:
            self.get_logger().info("📭 유효 각도 없음 → steering=0")
            self.pub.publish(Float32(data=0.0))
            return

        avg_angle = np.mean(angles)
        self.pub.publish(Float32(data=avg_angle))
        self.get_logger().info(f"✅ Steering angle (ROI avg): {avg_angle:.2f}°")

        # === 시각화 ===
        if self.visualize:
            vis_img = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
            for pt in filtered_pts:
                cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 2, (0, 255, 0), -1)
                cv2.line(vis_img, (int(ref[0]), int(ref[1])), (int(pt[0]), int(pt[1])), (50, 50, 255), 1)
            cv2.circle(vis_img, (int(ref[0]), int(ref[1])), 4, (255, 255, 255), -1)
            cv2.putText(vis_img, f"{avg_angle:.2f} deg", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow("Filtered ROI Angle", vis_img)
            cv2.waitKey(1)

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleLaneAngleEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
