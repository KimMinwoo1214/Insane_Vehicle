from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32
from cv_bridge import CvBridge
import numpy as np

class YOLOLaneEngineNode(Node):
    def __init__(self):
        super().__init__('yolo_lane_engine_node')
        self.bridge = CvBridge()

        # YOLO TensorRT 엔진 로드
        self.model = YOLO(
            "/home/parkm04/PycharmProjects/Insane_Vehicle/lane_ws/train1/weights/best.engine",
            task="segment"
        )

        self.subscription = self.create_subscription(
            Image, '/video_frames', self.callback, 10)
        self.polygon_pub = self.create_publisher(
            PolygonStamped, '/yolo_lane_polygon', 10)

        # ROI 하단 필터링 경계 설정
        self.roi_ymin = 0
        self.roi_ymax = 480

        self.get_logger().info("✅ YOLO Lane Engine 노드 시작됨")

    def callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # YOLO 추론
            results = self.model.predict(
                source=img, conf=0.25, iou=0.7)[0]

            if results.masks and results.masks.xy:
                for i, polygon in enumerate(results.masks.xy):
                    pts = np.array(polygon, dtype=np.float32)

                    # === ROI 필터링 ===
                    pts = pts[(pts[:, 1] >= self.roi_ymin) & (pts[:, 1] <= self.roi_ymax)]
                    if pts.shape[0] < 5:
                        continue  # 너무 적으면 무시

                    poly_msg = PolygonStamped()
                    poly_msg.header = msg.header
                    for x_pt, y_pt in pts:
                        pt32 = Point32(x=float(x_pt), y=float(y_pt), z=0.0)
                        poly_msg.polygon.points.append(pt32)

                    self.polygon_pub.publish(poly_msg)
                    self.get_logger().info(
                        f"📤 ROI Polygon {i} (pts: {pts.shape[0]}) 퍼블리시 완료")

        except Exception as e:
            self.get_logger().error(f"❌ 추론 또는 퍼블리시 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOLaneEngineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
