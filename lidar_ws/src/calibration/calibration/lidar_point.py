import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # LiDAR 토픽 (환경에 맞게 수정)
            self.lidar_callback,
            10)
        self.lidar_points = []

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # 특정 거리 범위 내에서 특징점 선택
        valid_indices = np.where((ranges > 0.1) & (ranges < 5.0))  # 10cm~5m 내 포인트만 사용
        x = ranges[valid_indices] * np.cos(angles[valid_indices])
        y = ranges[valid_indices] * np.sin(angles[valid_indices])

        self.lidar_points = list(zip(x, y))  # (x, y) 저장

        # 파일 저장
        with open("data/laser_points.txt", "w") as f:
            for px, py in self.lidar_points:
                f.write(f"{px} {py}\n")

        self.get_logger().info("LiDAR points saved!")

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
