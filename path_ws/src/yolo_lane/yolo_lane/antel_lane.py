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
        self.angle_margin_deg = 2.0     # 허용 오차 각도 (degree)
        self.border_margin_px = 10      # 이미지 경계에서 이내의 직선은 제거

        # ===== ROS 설정 =====
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # PolygonStamped 구독자로 변경!
        self.sub = self.create_subscription(
            PolygonStamped,
            '/yolo_polygon',
            self.callback,
            qos
        )
        self.pub = self.create_publisher(Float32, '/lane_steering_angle', qos)

    def callback(self, msg):
        # PolygonStamped는 하나의 polygon을 포함
        if len(msg.polygon.points) < 3:
            return

        # 폴리곤 points → NumPy array
        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.int32)

        # 이미지 사이즈 임의 지정 (예: 640x480)
        w, h = 640, 480
        edge_img = np.zeros((h, w), dtype=np.uint8)

        # 폴리라인 그리기
        cv2.polylines(edge_img, [pts], isClosed=True, color=255, thickness=2)

        # Dominant gradient 계산
        gradient = self.compute_dominant_gradient(edge_img, w, h)

        if gradient is not None:
            angle = (gradient + 90)  # -90 ~ 90 → 0 ~ 180
            angle = np.clip(angle, 0, 180)
            msg_out = Float32()
            msg_out.data = float(angle)
            self.pub.publish(msg_out)
            self.get_logger().info(f"Steering angle: {angle:.2f}°")

            # 시각화
            cv2.imshow("Lane edge", edge_img)
            cv2.waitKey(1)

    def compute_dominant_gradient(self, image, w, h):
        if image.dtype != np.uint8:
            image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')

        lines = cv2.HoughLines(image, 1, np.pi / 180, 80)
        if lines is None:
            return None

        margin_rad = np.deg2rad(self.angle_margin_deg)
        valid_angles = []

        for line in lines:
            rho, theta = line[0]

            # 이미지 경계 근처에 있는 직선 제거
            is_near_border = (
                abs(rho) < self.border_margin_px or
                abs(rho - w) < self.border_margin_px or
                abs(rho - h) < self.border_margin_px
            )

            is_horizontal = abs(theta - 0) < margin_rad or abs(theta - np.pi) < margin_rad
            is_vertical = abs(theta - np.pi / 2) < margin_rad

            if (is_horizontal or is_vertical) and is_near_border:
                continue

            angle_deg = np.rad2deg(np.arctan2(np.sin(theta), np.cos(theta)))
            valid_angles.append(angle_deg)

        if len(valid_angles) == 0:
            return None

        return np.median(valid_angles)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLaneAngleEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

