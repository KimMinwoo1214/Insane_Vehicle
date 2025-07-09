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

        # ROI 상단 y값
        self.roi_ymin = 240

        # slicing bin 세팅
        self.slice_step = 10         # y 슬라이스 간격 (픽셀)
        self.slice_bin = 10          # bin 폭 (픽셀)
        self.min_points_per_bin = 1  # 최소 점 개수: vertex sparse할 때는 1로 둬야 끊김 방지

        # 곡선 근사
        self.polyfit_degree = 3      # 다항식 차수: 일반적으로 2~3차

        # tangent angle 계산할 슬라이스 개수
        self.num_bottom_slices = 3   # ROI 하단 몇 슬라이스로 tangent 평균 낼지

        # 외곽선 단순화: 너무 울퉁불퉁한 테두리 smooth 용도
        self.approx_epsilon = 5.0

        # ===== 오프셋 & 게인 파라미터 =====

        # lateral offset tuning (픽셀): 차선 중심선이 차폭 중심과 다르면 여기서 보정
        self.centerline_offset_px = 0.0

        # steering gain scaling: tangent로 구한 각도를 실차에 맞게 보정
        self.steering_gain = 1.0

        # 이미지 크기
        self.img_w = 640
        self.img_h = 480

        qos = rclpy.qos.QoSProfile(depth=1)

        self.sub = self.create_subscription(
            PolygonStamped,
            '/engine_lane_polygon',
            self.callback,
            qos
        )
        self.pub = self.create_publisher(Float32, '/lane_steering_angle', qos)

        self.get_logger().info(
            "✅ SimpleLaneAngleEstimator: slicing + polyfit + tangent 평균 + offset/gain tuning 시작!"
        )

    def callback(self, msg):
        if len(msg.polygon.points) < 3:
            self.get_logger().info("📭 Polygon point 부족")
            return

        # ===== Polygon → NumPy =====
        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.int32)

        # ===== ROI 필터 =====
        roi_pts = pts[pts[:, 1] >= self.roi_ymin]
        if len(roi_pts) < 3:
            self.get_logger().info("📭 ROI 내 유효 점 부족")
            return

        # ===== contour 단순화 =====
        contour = roi_pts.reshape((-1, 1, 2)).astype(np.int32)
        approx = cv2.approxPolyDP(contour, self.approx_epsilon, True)
        approx_pts = approx.reshape(-1, 2)

        # ===== Edge mask =====
        edge_mask = np.zeros((self.img_h, self.img_w), dtype=np.uint8)
        cv2.polylines(edge_mask, [approx_pts], isClosed=True, color=255, thickness=2)

        # ===== 조밀한 contour =====
        contours, _ = cv2.findContours(edge_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 0:
            self.get_logger().info("📭 조밀한 contour 없음")
            return
        contour_pts = contours[0].reshape(-1, 2)

        cv2.imshow("Polygon Edge (approx)", edge_mask)

        # ===== slicing =====
        y_min, y_max = np.min(contour_pts[:, 1]), np.max(contour_pts[:, 1])
        y_slices = np.arange(y_min, y_max, self.slice_step)

        centerline_pts = []
        for y in y_slices:
            bin_mask = (contour_pts[:, 1] >= y - self.slice_bin / 2) & (contour_pts[:, 1] < y + self.slice_bin / 2)
            bin_pts = contour_pts[bin_mask]

            if len(bin_pts) < self.min_points_per_bin:
                continue

            xs = bin_pts[:, 0]
            left_x = np.min(xs)
            right_x = np.max(xs)
            center_x = (left_x + right_x) / 2.0

            # ===== lateral offset 적용 =====
            center_x += self.centerline_offset_px  # 픽셀 단위로 중심선 오프셋 조정

            centerline_pts.append([center_x, y])

        if len(centerline_pts) < 5:
            self.get_logger().info("📭 slicing 결과 유효 중점 부족")
            return

        centerline_pts = np.array(centerline_pts)

        # ===== polyfit =====
        fit = np.polyfit(centerline_pts[:, 1], centerline_pts[:, 0], self.polyfit_degree)
        poly = np.poly1d(fit)

        # ===== 시각화 =====
        y_fit = np.linspace(y_min, y_max, 100)
        x_fit = poly(y_fit)
        edge_color = cv2.cvtColor(edge_mask, cv2.COLOR_GRAY2BGR)
        for x, y in centerline_pts:
            cv2.circle(edge_color, (int(x), int(y)), 2, (0, 255, 0), -1)
        for i in range(len(y_fit) - 1):
            pt1 = (int(x_fit[i]), int(y_fit[i]))
            pt2 = (int(x_fit[i + 1]), int(y_fit[i + 1]))
            cv2.line(edge_color, pt1, pt2, (0, 0, 255), 1)

        cv2.imshow("Centerline Polyfit (contour)", edge_color)
        cv2.waitKey(1)

        # ===== tangent angle (최하단 N슬라이스 평균) =====
        bottom_slices = centerline_pts[:self.num_bottom_slices, 1]
        tangent_angles = []
        for y0 in bottom_slices:
            dy = 1.0
            dx = (poly(y0 + dy) - poly(y0 - dy)) / (2 * dy)
            angle_rad = np.arctan(dx)
            angle_deg = np.rad2deg(angle_rad)
            tangent_angles.append(angle_deg)

        avg_tangent = np.mean(tangent_angles)

        # ===== steering gain scaling =====
        steering_angle = 90.0 + (avg_tangent * self.steering_gain)

        self.pub.publish(Float32(data=steering_angle))
        self.get_logger().info(
            f"✅ Steering angle: {steering_angle:.2f}° "
            f"(tangent avg: {avg_tangent:.2f}°, offset_px={self.centerline_offset_px}, gain={self.steering_gain})"
        )

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

