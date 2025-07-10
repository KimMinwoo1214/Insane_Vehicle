#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32
from cv_bridge import CvBridge
import cv2
import numpy as np

class YOLOSegEngineNode(Node):
    def __init__(self):
        super().__init__('yolo_seg_engine_node')
        self.bridge = CvBridge()

        # ✅ TensorRT YOLO 엔진 모델 로드 (.engine + task="segment")
        self.model = YOLO(
            "/home/parkm04/PycharmProjects/Insane_Vehicle/lane_ws/train9/weights/best.engine",
            task="segment"
        )

        self.subscription = self.create_subscription(
            Image,
            '/video_frames',
            self.callback,
            10
        )

        self.polygon_pub = self.create_publisher(PolygonStamped, '/yolo_polygon', 10)

        self.get_logger().info("✅ YOLOSegEngineNode 시작됨 (/video_frames → /yolo_polygon)")

    def callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # ✅ YOLO 추론 (TensorRT 엔진)
            results = self.model(img)[0]

            largest_polygon = None
            max_area = 0

            if results.masks is not None and results.masks.xy is not None:
                for polygon in results.masks.xy:
                    contour = np.array(polygon, dtype=np.int32)
                    area = cv2.contourArea(contour)
                    if area > max_area:
                        max_area = area
                        largest_polygon = polygon

                if largest_polygon is not None:
                    poly_msg = PolygonStamped()
                    poly_msg.header = msg.header
                    for pt in largest_polygon:
                        x, y = float(pt[0]), float(pt[1])
                        poly_msg.polygon.points.append(Point32(x=x, y=y, z=0.0))
                    self.polygon_pub.publish(poly_msg)
                    self.get_logger().info(f"📤 가장 큰 Polygon 퍼블리시 완료 (points={len(largest_polygon)}, area={max_area:.1f})")
                else:
                    self.get_logger().info("📭 유효한 Polygon 없음")

            # ✅ 시각화 (YOLO segmentation 결과)
            result_img = results.plot()
            cv2.imshow("YOLO Segmentation (TensorRT)", result_img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ 추론/퍼블리시 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOSegEngineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
