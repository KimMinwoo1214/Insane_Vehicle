from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2

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
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # YOLO는 RGB 형식 사용
            white_lower = np.array([0, 0, 200], dtype=np.uint8)
            white_upper = np.array([180, 30, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, white_lower, white_upper)
            img = cv2.bitwise_and(img, img, mask=mask)

            # YOLO 세그멘테이션 추론
            results = self.model(img)[0]


            # polygon 추출 및 퍼블리시
            if results.masks is not None and results.masks.xy is not None:
                for i, polygon in enumerate(results.masks.xy):
                    poly_msg = PolygonStamped()
                    poly_msg.header = msg.header
                    for pt in polygon:
                        x, y = float(pt[0]), float(pt[1])
                        poly_msg.polygon.points.append(Point32(x=x, y=y, z=0.0))
                    self.polygon_pub.publish(poly_msg)
                    self.get_logger().info(f"📤 Polygon {i} 퍼블리시 완료 ({len(polygon)} points)")

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

