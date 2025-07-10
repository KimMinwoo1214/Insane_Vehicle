#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import PolygonStamped
import numpy as np
import cv2

class EfficientHybridAngleEstimator(Node):
    def __init__(self):
        super().__init__('efficient_hybrid_angle_node')

        # ===== 하이퍼파라미터 =====
        self.img_w = 640
        self.img_h = 480
        self.slice_interval = 10
        self.num_slices = 9
        self.minx = 150
        self.maxx = 490

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

        self.get_logger().info("✅ Efficient Hybrid Angle Estimator 시작됨")

    def callback(self, msg):
        if len(msg.polygon.points) < 3:
            self.pub.publish(Float32(data=0.0))
            return

        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.float32)

        # === 기준 y 리스트 만들기 (하단 slice 기준)
        y_base = self.img_h - self.slice_interval * self.num_slices
        y_slices = [y_base + i * self.slice_interval for i in range(self.num_slices)]

        # === 각 슬라이스에 속하는 점 누적 (한 번의 루프)
        bins = {y: [] for y in y_slices}
        half = self.slice_interval / 2
        for pt in pts:
            y = pt[1]
            for y_target in y_slices:
                if abs(y - y_target) < half:
                    bins[y_target].append(pt)
                    break

        # === 각 슬라이스별 평균점 계산
        ref_pts = []
        for y in y_slices:
            bin_pts = bins[y]
            if len(bin_pts) >= 2:
                mean_pt = np.mean(bin_pts, axis=0)
                ref_pts.append(mean_pt)

        if len(ref_pts) < 2:
            self.pub.publish(Float32(data=0.0))
            return

        ref_pts = np.array(ref_pts)

        # === 최하단 기준점 (가장 y 큰 ref_pt)
        base_pt = ref_pts[-1]

        # === 각도 계산
        angles = []
        for pt in ref_pts[:-1]:
            dx = pt[0] - base_pt[0]
            dy = base_pt[1] - pt[1]
            if dy == 0:
                continue
            angle_rad = np.arctan2(dx, dy)
            angles.append(90.0 + np.rad2deg(angle_rad))

        if len(angles) == 0:
            self.pub.publish(Float32(data=0.0))
            return

        final_angle = np.mean(angles)

        # === 좌우 보정
        if base_pt[0] < self.minx:
            final_angle -= 10.0
            self.get_logger().info("🔄 최하단 기준점 좌측 → 좌측으로 보정 (-10°)")
        elif base_pt[0] > self.maxx:
            final_angle += 10.0
            self.get_logger().info("🔄 최하단 기준점 우측 → 우측으로 보정 (+10°)")

        self.pub.publish(Float32(data=final_angle))
        self.get_logger().info(f"✅ 조향각 계산 완료: {final_angle:.2f}°")

        # === 시각화 ===
        if self.visualize:
            vis_img = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
            for pt in ref_pts:
                cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 3, (0, 255, 0), -1)
                cv2.line(vis_img, tuple(base_pt.astype(int)), tuple(pt.astype(int)), (0, 255, 255), 1)
            cv2.circle(vis_img, tuple(base_pt.astype(int)), 5, (0, 0, 255), -1)
            cv2.putText(vis_img, f"{final_angle:.2f} deg", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow("Efficient Hybrid Angle", vis_img)
            cv2.waitKey(1)

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = EfficientHybridAngleEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
