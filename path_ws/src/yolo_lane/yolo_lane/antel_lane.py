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
        self.roi_ymin = 240  # ROI 상단 y좌표 (하단은 480)
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
        self.get_logger().info("✅ ROI 기반 조향각 노드 시작!")

    def callback(self, msg):
        if len(msg.polygon.points) < 2:
            self.pub.publish(Float32(data=0.0))
            return

        # Polygon을 numpy 배열로 변환
        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.float32)

        # === ROI 필터링 ===
        roi_pts = pts[pts[:, 1] >= self.roi_ymin]
        if len(roi_pts) < 2:
            self.get_logger().info("📭 ROI 내 점 부족 → steering=0")
            self.pub.publish(Float32(data=0.0))
            return

        # 최하단 점 (y가 가장 큰 점)
        bottom = roi_pts[np.argmax(roi_pts[:, 1])]

        # 기울기(각도) 계산: bottom 기준, 나머지 점들과
        angles = []
        for pt in roi_pts:
            if np.all(pt == bottom):
                continue
            dx = pt[0] - bottom[0]
            dy = bottom[1] - pt[1]  # 아래에서 위로 올라가야 양수
            if dy == 0:
                continue
            angle_rad = np.arctan2(dx, dy)
            angle_deg = 90.0 + np.rad2deg(angle_rad)  # 왼쪽 0, 정면 90, 오른쪽 180
            angles.append(angle_deg)

        if len(angles) == 0:
            self.get_logger().info("📭 유효 기울기 없음 → steering=0")
            self.pub.publish(Float32(data=0.0))
            return

        avg_angle = np.mean(angles)
        self.pub.publish(Float32(data=avg_angle))
        self.get_logger().info(f"✅ Steering angle (ROI avg): {avg_angle:.2f}°")

        # ===== 시각화 =====
        if self.visualize:
            vis_img = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
            for pt in roi_pts:
                cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 2, (0, 255, 0), -1)
            cv2.putText(vis_img, f"{avg_angle:.2f} deg", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow("ROI Mean Angle", vis_img)
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
