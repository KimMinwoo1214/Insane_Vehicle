#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32
from cv_bridge import CvBridge
import cv2
import numpy as np
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline


class EngineLaneNode(Node):
    def __init__(self):
        super().__init__('engine_lane_node')
        self.bridge = CvBridge()

        # ✅ TensorRT YOLO 엔진 로드
        self.model = YOLO("/home/parkm04/PycharmProjects/Insane_Vehicle/lane_ws/train9/weights/best.engine", task="segment")

        self.subscription = self.create_subscription(
            Image,
            '/video_frames',
            self.callback,
            10
        )

        self.polygon_pub = self.create_publisher(PolygonStamped, '/engine_lane_polygon', 10)

        # ROI (하단만 쓰고 싶으면 범위 조절)
        self.roi_ymin = 200
        self.roi_ymax = 480

        self.get_logger().info("✅ EngineLaneNode 시작됨 (/video_frames → /engine_lane_polygon)")

    def callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(img)[0]

            # ✅ 1) 가장 큰 polygon 선택
            largest_polygon = None
            max_area = 0

            if results.masks is not None and results.masks.xy is not None:
                for polygon in results.masks.xy:
                    contour = np.array(polygon, dtype=np.int32)
                    area = cv2.contourArea(contour)
                    if area > max_area:
                        max_area = area
                        largest_polygon = contour

            if largest_polygon is None or len(largest_polygon) < 3:
                self.get_logger().info("📭 유효한 polygon 없음")
                return

            # ✅ 2) ROI 안에 있는 꼭짓점만 사용
            roi_mask = (largest_polygon[:, 1] > self.roi_ymin) & (largest_polygon[:, 1] < self.roi_ymax)
            pts_roi = largest_polygon[roi_mask]

            if len(pts_roi) < 5:
                self.get_logger().info("📭 ROI 내 유효 점 부족")
                return

            X = pts_roi[:, 1].reshape(-1, 1)  # y축
            y = pts_roi[:, 0]                 # x축

            # ✅ 3) RANSAC 2차 곡선 근사
            model = make_pipeline(PolynomialFeatures(2), RANSACRegressor(residual_threshold=5.0))
            model.fit(X, y)
            inlier_mask = model.named_steps['ransacregressor'].inlier_mask_
            inlier_pts = pts_roi[inlier_mask]

            if len(inlier_pts) < 5:
                self.get_logger().info("📭 RANSAC Inlier 부족")
                return

            # ✅ 4) PolygonStamped 퍼블리시
            poly_msg = PolygonStamped()
            poly_msg.header = msg.header
            for x, y in zip(inlier_pts[:, 0], inlier_pts[:, 1]):
                poly_msg.polygon.points.append(Point32(x=float(x), y=float(y), z=0.0))

            self.polygon_pub.publish(poly_msg)
            self.get_logger().info(f"📤 가장 큰 polygon + RANSAC 퍼블리시 (inliers={len(inlier_pts)}, area={max_area:.1f})")

            # ✅ 5) 시각화
            #mask_img = img.copy()
            #for pt in inlier_pts:
            #    cv2.circle(mask_img, (int(pt[0]), int(pt[1])), 3, (0, 255, 0), -1)
            #cv2.imshow("RANSAC Inlier", mask_img)
            #cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ 추론/퍼블리시 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = EngineLaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

