import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np

class LidarCameraOverlay(Node):
    def __init__(self):
        super().__init__('lidar_camera_overlay')
        self.bridge = CvBridge()

        # ROS2 카메라 & LiDAR 데이터 구독
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Homography 변환 행렬 로드
        self.H = np.loadtxt("data/H_matrix.txt")

        # 저장된 LiDAR 데이터
        self.lidar_points = []

    def lidar_callback(self, msg):
        points = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        for i, r in enumerate(msg.ranges):
            if 0.1 < r < 10.0:  # 10cm ~ 10m 범위만 변환
                angle = angle_min + i * angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)

                # Homogeneous 좌표 변환
                lidar_point = np.array([x, y, 1.0])
                image_point = self.H @ lidar_point
                image_point /= image_point[2]  # 정규화
                points.append(image_point[:2])

        self.lidar_points = np.array(points)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # LiDAR 포인트를 카메라 이미지에 오버레이
        for point in self.lidar_points:
            u, v = int(point[0]), int(point[1])
            if 0 <= u < frame.shape[1] and 0 <= v < frame.shape[0]:  # 이미지 내부에 있을 때만 표시
                cv2.circle(frame, (u, v), 5, (0, 0, 255), -1)

        # 결과 화면 표시
        cv2.imshow("Lidar to Camera Overlay", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraOverlay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

