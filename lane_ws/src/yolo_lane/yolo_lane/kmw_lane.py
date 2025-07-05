from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline

class YOLOSegNode(Node):
    def __init__(self):
        super().__init__('yolo_seg_node')
        self.bridge = CvBridge()

        # YOLOv8/11 세그멘테이션 모델 로드
        self.model = YOLO("train1/weights/best.pt")  # 경로 수정 가능

        # 구독: 이미지 입력
        self.subscription = self.create_subscription(
            Image,
            '/video_frames',
            self.callback,
            10)

        # 퍼블리셔: polygon 좌표
        self.polygon_pub = self.create_publisher(PolygonStamped, '/yolo_polygon', 10)

        self.get_logger().info("✅ YOLO Segmentation 노드 시작됨 (/video_frames 구독, /yolo_polygon 퍼블리시)")

    def callback(self, msg):
        try:
            # ROS 이미지 → OpenCV 이미지로 변환
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            white_lower = np.array([0, 0, 200], dtype=np.uint8)
            white_upper = np.array([180, 30, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, white_lower, white_upper)
            img = cv2.bitwise_and(cv_img, cv_img, mask=mask)

            # YOLO 세그멘테이션 추론
            results = self.model(img)[0]

            # polygon 추출 및 RANSAC 노이즈 제거 후 퍼블리시
            if results.masks and results.masks.xy:
                for i, polygon in enumerate(results.masks.xy):
                    pts = np.array(polygon, dtype=np.float32)  # shape (N,2)
                    if pts.shape[0] < 10:
                        continue
                    # RANSAC: y -> x polynomial fitting
                    X = pts[:,1].reshape(-1,1)  # y 값
                    y_vals = pts[:,0]           # x 값
                    # pipeline: 2차 다항식 + RANSAC
                    model = make_pipeline(PolynomialFeatures(degree=2),
                                           RANSACRegressor(residual_threshold=5.0,
                                                           max_trials=100))
                    model.fit(X, y_vals)
                    inlier_mask = model.named_steps['ransacregressor'].inlier_mask_
                    inliers = pts[inlier_mask]

                    # PolygonStamped 메시지에 inliers만 퍼블리시
                    poly_msg = PolygonStamped()
                    poly_msg.header = msg.header
                    for x_pt, y_pt in inliers:
                        pt32 = Point32(x=float(x_pt), y=float(y_pt), z=0.0)
                        poly_msg.polygon.points.append(pt32)
                    self.polygon_pub.publish(poly_msg)
                    self.get_logger().info(f"📤 Polygon {i} (inliers: {inliers.shape[0]}) 퍼블리시 완료")

            # (선택) 결과 시각화
            result_img = results.plot()
            cv2.imshow("YOLO Segmentation", result_img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ 추론 또는 퍼블리시 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YOLOSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
