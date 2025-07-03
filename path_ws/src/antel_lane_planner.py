import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneAngleEstimatorNode(Node):
    def __init__(self):
        super().__init__('lane_angle_estimator')

        # ================== Hyperparameters ===================
        self.declare_parameter('sub_topic', '/yolo_polygon')  # 구독할 polygon 토픽 이름
        self.declare_parameter('pub_topic', '/lane_steering_angle')
        self.declare_parameter('roi_image_topic', '/roi_image')
        self.declare_parameter('show_image', True)
        self.declare_parameter('lane_width', 300)  # BEV 상 차선 폭(px)
        self.declare_parameter('cutting_idx', 300)  # ROI 자를 y 위치(px)
        self.declare_parameter('src_mat', [[238, 316],[402, 313], [501, 476], [155, 476]])  # 원본 이미지상의 사각형 꼭짓점
        self.declare_parameter('dst_mat', [[192, 0], [448, 0], [448, 480], [192, 480]])  # BEV 변환 후 사각형의 위치

        self.sub_topic = self.get_parameter('sub_topic').value
        self.pub_topic = self.get_parameter('pub_topic').value
        self.roi_topic = self.get_parameter('roi_image_topic').value
        self.show_image = self.get_parameter('show_image').value
        self.lane_width = self.get_parameter('lane_width').value
        self.cutting_idx = self.get_parameter('cutting_idx').value
        self.src_mat = np.float32(self.get_parameter('src_mat').value)
        self.dst_mat = np.float32(self.get_parameter('dst_mat').value)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscriber = self.create_subscription(DetectionArray, self.sub_topic, self.callback, qos_profile)
        self.angle_publisher = self.create_publisher(Float32, self.pub_topic, qos_profile)
        self.roi_publisher = self.create_publisher(Image, self.roi_topic, qos_profile)

        self.bridge = CvBridge()

    def callback(self, msg):
        if len(msg.detections) == 0:
            return

        edge_img = self.draw_edges(msg, 'lane', color=255)
        bev_img = self.bird_convert(edge_img)
        roi_img = bev_img[self.cutting_idx:]

        gradient = self.dominant_gradient(roi_img, theta_limit=70)
        steering_angle = 90.0 + gradient

        self.angle_publisher.publish(Float32(data=steering_angle))

        # ROI 이미지 퍼블리시
        try:
            roi_msg = self.bridge.cv2_to_imgmsg(roi_img.astype(np.uint8), encoding='mono8')
            self.roi_publisher.publish(roi_msg)
        except Exception as e:
            self.get_logger().error(f"ROI image publish failed: {e}")

        if self.show_image:
            cv2.imshow("ROI Image", roi_img)
            cv2.waitKey(1)

    def draw_edges(self, detection_msg, cls_name, color):
        h = detection_msg.detections[0].mask.height
        w = detection_msg.detections[0].mask.width
        edge_img = np.zeros((h, w), dtype=np.uint8)

        for det in detection_msg.detections:
            if det.class_name == cls_name:
                mask = np.array([[int(p.x), int(p.y)] for p in det.mask.data])
                if mask.shape[0] > 2:
                    cv2.polylines(edge_img, [mask], True, color, 1)
        return edge_img

    def bird_convert(self, img):
        h, w = img.shape[:2]
        transform = cv2.getPerspectiveTransform(self.src_mat, self.dst_mat)
        warped = cv2.warpPerspective(img, transform, (w, h))
        return warped

    def dominant_gradient(self, image, theta_limit):
        if image.dtype != np.uint8:
            image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')

        lines = cv2.HoughLines(image, 1, np.pi / 180, 80)
        if lines is None:
            return 0.0

        angles = []
        right = np.deg2rad(90 + (90 - theta_limit))
        left = np.deg2rad(90 - (90 - theta_limit))
        epsilon = np.deg2rad(5)  # 수평선 근처 (0°, 180°) 제외용

        for line in lines:
            rho, theta = line[0]

            # 수직에 가까운 선들만 허용, 수평선 근처 제거
            if (left < theta < right) and not (abs(theta) < epsilon or abs(theta - np.pi) < epsilon):
                angle = np.rad2deg(np.arctan2(np.sin(theta), np.cos(theta)))
                angles.append(angle)

        return np.median(angles) if angles else 0.0


def main(args=None):
    rclpy.init(args=args)
    node = LaneAngleEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
