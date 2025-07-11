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

        self.config = {
            'img_w': 640,
            'img_h': 480,
            'roi_ymin': 300,
            'roi_ymax': 400,
            'slice_step': 10,
            'visualize': True,
            'num_bottom_slices': 3,  # 벡터 평균에 사용할 하단 인접 쌍 수
        }

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

        self.get_logger().info("✅ HybridTargetSliceAngleEstimator (벡터기반) 시작됨")

    def callback(self, msg):
        if len(msg.polygon.points) < 3:
            self.pub.publish(Float32(data=0.0))
            return

        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.int32)
        mask = np.zeros((self.config['img_h'], self.config['img_w']), dtype=np.uint8)
        cv2.polylines(mask, [pts.reshape(-1, 1, 2)], isClosed=True, color=255, thickness=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            self.pub.publish(Float32(data=0.0))
            return

        contour = max(contours, key=cv2.contourArea)
        contour_pts = contour.reshape(-1, 2)

        # === ROI 영역에서 슬라이싱 후 중앙점 추출 ===
        sliced_lanes = {}
        for x, y in contour_pts:
            if self.config['roi_ymin'] <= y <= self.config['roi_ymax']:
                slice_idx = (y - self.config['roi_ymin']) // self.config['slice_step']
                if slice_idx not in sliced_lanes:
                    sliced_lanes[slice_idx] = []
                sliced_lanes[slice_idx].append(x)

        target_pts = []
        for slice_idx in sorted(sliced_lanes.keys(), reverse=True):  # 하단부터
            x_list = sliced_lanes[slice_idx]
            if len(x_list) < 2:
                continue
            center_x = (np.min(x_list) + np.max(x_list)) / 2.0
            center_y = self.config['roi_ymin'] + slice_idx * self.config['slice_step'] + self.config['slice_step'] / 2
            target_pts.append([center_x, center_y])

        if len(target_pts) < self.config['num_bottom_slices'] + 1:
            self.pub.publish(Float32(data=0.0))
            return

        target_pts = np.array(target_pts)

        # === 벡터 각도 계산 ===
        bottom_pts = target_pts[:self.config['num_bottom_slices'] + 1]
        angles = []
        for i in range(len(bottom_pts) - 1):
            dx = bottom_pts[i + 1][0] - bottom_pts[i][0]
            dy = bottom_pts[i + 1][1] - bottom_pts[i][1]
            if abs(dy) < 1e-6:
                continue
            angle_rad = np.arctan2(dx, dy)
            angle_deg = np.rad2deg(angle_rad)

            # 0~180도 범위로 정규화
            angle_deg = (angle_deg + 360) % 360
            angle_deg = angle_deg if angle_deg <= 180 else 360 - angle_deg
            angles.append(angle_deg)

        if len(angles) == 0:
            self.pub.publish(Float32(data=0.0))
            return

        final_angle = np.mean(angles)
        self.pub.publish(Float32(data=final_angle))
        self.get_logger().info(f"✅ Steering angle (vector-based): {final_angle:.2f}°")

        # === 시각화 ===
        if self.config['visualize']:
            try:
                vis = np.zeros((self.config['img_h'], self.config['img_w'], 3), dtype=np.uint8)
                for pt in target_pts:
                    pt_int = tuple(np.round(pt).astype(int))
                    cv2.circle(vis, pt_int, 3, (0, 255, 0), -1)
                for i in range(len(bottom_pts) - 1):
                    pt1 = tuple(np.round(bottom_pts[i]).astype(int))
                    pt2 = tuple(np.round(bottom_pts[i + 1]).astype(int))
                    cv2.line(vis, pt1, pt2, (0, 255, 255), 1)

                cv2.putText(vis, f"{final_angle:.2f} deg", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow("Vector Angle ROI", vis)
                cv2.waitKey(1)
            except cv2.error:
                self.get_logger().warn("시각화 실패. 비활성화합니다.")
                self.config['visualize'] = False

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
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

