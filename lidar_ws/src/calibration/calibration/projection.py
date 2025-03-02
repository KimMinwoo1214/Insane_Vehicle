import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class LidarProjection(Node):
    def __init__(self):
        super().__init__('lidar_projection')
        self.bridge = CvBridge()

        # 카메라 & LiDAR 변환 데이터 구독
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(Float32MultiArray, '/lidar_to_camera', self.lidar_callback, 10)

        self.lidar_points = []

    def lidar_callback(self, msg):
        # 수신된 LiDAR 데이터 (u, v) 좌표
        data = np.array(msg.data).reshape(-1, 2)
        self.lidar_points = data

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # LiDAR 포인트를 이미지 위에 그리기
        for u, v in self.lidar_points:
            if 0 <= u < frame.shape[1] and 0 <= v < frame.shape[0]:  # 이미지 내부에 있는 경우만 그림
                cv2.circle(frame, (int(u), int(v)), 5, (0, 0, 255), -1)

        cv2.imshow("Lidar Projection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LidarProjection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
