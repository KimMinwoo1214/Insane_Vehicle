import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

class LidarToCameraNode(Node):
    def __init__(self):
        super().__init__('lidar_to_camera')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # LiDAR 데이터 토픽
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/lidar_to_camera',  # 변환된 데이터 퍼블리시할 토픽
            10)
        
        # Homography 행렬 로드
        self.H = np.loadtxt("data/H_matrix.txt")  # 저장된 행렬 로드

    def lidar_callback(self, msg):
        points = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        for i, r in enumerate(msg.ranges):
            if 0.1 < r < 10.0:  # 10cm ~ 10m 범위만 사용
                angle = angle_min + i * angle_increment
                x = r * np.cos(angle)  # LiDAR x 좌표
                y = r * np.sin(angle)  # LiDAR y 좌표

                # Homogeneous 좌표 변환
                lidar_point = np.array([x, y, 1.0])
                image_point = self.H @ lidar_point
                image_point /= image_point[2]  # 정규화
                
                # 변환된 (u, v) 좌표 추가
                points.append(image_point[:2])

        # 메시지 생성 후 퍼블리시
        msg_out = Float32MultiArray()
        msg_out.data = np.array(points).flatten().tolist()  # 1D 리스트로 변환
        self.publisher.publish(msg_out)
        self.get_logger().info("📌 변환된 LiDAR 데이터를 카메라 좌표계로 퍼블리시함!")

def main(args=None):
    rclpy.init(args=args)
    node = LidarToCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
