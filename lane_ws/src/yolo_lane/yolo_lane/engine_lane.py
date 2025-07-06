from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32
from cv_bridge import CvBridge
import numpy as np
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline

class YOLOLaneEngineNode(Node):
    def __init__(self):
        super().__init__('yolo_lane_engine_node')
        self.bridge = CvBridge()

        # TensorRT 엔진 파일을 segmentation 모델로 로드 (task 지정 필수)
        self.model = YOLO(
            "/home/parkm04/PycharmProjects/Insane_Vehicle/lane_ws/train1/weights/best.engine",
            task="segment"
        )

        self.subscription = self.create_subscription(
            Image, '/video_frames', self.callback, 10)
        self.polygon_pub = self.create_publisher(
            PolygonStamped, '/yolo_lane_polygon', 10)

        self.get_logger().info("✅ YOLO Lane Engine 노드 시작됨")

    def callback(self, msg):
        # 예외 발생 시 로그 출력만 하고 리턴
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 명시적 predict 호출
            results = self.model.predict(
                source=img, conf=0.25, iou=0.7)[0]

            if results.masks and results.masks.xy:
                for i, polygon in enumerate(results.masks.xy):
                    pts = np.array(polygon, dtype=np.float32)
                    if pts.shape[0] < 10:
                        continue

                    X = pts[:,1].reshape(-1,1)
                    y_vals = pts[:,0]
                    pipeline = make_pipeline(
                        PolynomialFeatures(degree=2),
                        RANSACRegressor(residual_threshold=5.0, max_trials=100)
                    )
                    try:
                        pipeline.fit(X, y_vals)
                        mask = pipeline.named_steps['ransacregressor'].inlier_mask_
                        inliers = pts[mask]
                    except ValueError:
                        inliers = pts  # 실패 시 원본 사용

                    poly_msg = PolygonStamped()
                    poly_msg.header = msg.header
                    for x_pt, y_pt in inliers:
                        pt32 = Point32(x=float(x_pt), y=float(y_pt), z=0.0)
                        poly_msg.polygon.points.append(pt32)
                    self.polygon_pub.publish(poly_msg)
                    self.get_logger().info(
                        f"📤 Polygon {i} (inliers: {inliers.shape[0]}) 퍼블리시 완료"
                    )

            # (선택) 결과 시각화
            # result_img = results.plot()
            # cv2.imshow("YOLO Lane Engine", result_img)
            # cv2.waitKey(1)

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
