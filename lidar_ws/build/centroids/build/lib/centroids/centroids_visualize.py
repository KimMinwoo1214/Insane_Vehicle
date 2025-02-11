import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

class LidarSideVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_side_visualizer')

        # `/scan` 토픽 구독
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.process_scan,
            10
        )

        # 왼쪽(L) 선을 위한 퍼블리셔
        self.left_publisher = self.create_publisher(
            Marker,
            '/left_path',
            10
        )

        # 오른쪽(R) 선을 위한 퍼블리셔
        self.right_publisher = self.create_publisher(
            Marker,
            '/right_path',
            10
        )

    def process_scan(self, msg):
        angle_min = msg.angle_min  # 최소 각도 (rad)
        angle_increment = msg.angle_increment  # 각도 증가량 (rad)
        ranges = msg.ranges  # 거리 데이터 (m)

        left_points = []
        right_points = []

        for i, r in enumerate(ranges):
            if r < 0.1 or r > 10:  # 10m 이상의 값은 무시
                continue

            theta = angle_min + (i * angle_increment)  # 현재 포인트의 각도 (rad)
            theta_deg = np.degrees(theta)  # 도(degree) 변환

            x = r * np.cos(theta)
            y = r * np.sin(theta)

            if -180 <= theta_deg <= -90:  # 왼쪽 (0° ~ 90°)
                left_points.append((x, y))
            elif 90 <= theta_deg <= 180:  # 오른쪽 (270° ~ 360°)
                right_points.append((x, y))

        # 왼쪽(L) 선 그리기
        self.publish_marker(self.left_publisher, left_points, "left_path", 0.0, 1.0, 0.0)  # 초록색

        # 오른쪽(R) 선 그리기
        self.publish_marker(self.right_publisher, right_points, "right_path", 0.0, 0.0, 1.0)  # 파란색

    def publish_marker(self, publisher, points, ns, r, g, b):
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # 선 두께

        # 색상 설정
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b

        for x, y in points:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            marker.points.append(point)

        publisher.publish(marker)
        self.get_logger().info(f"Published {ns} path with {len(points)} points.")

def main(args=None):
    rclpy.init(args=args)
    node = LidarSideVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

