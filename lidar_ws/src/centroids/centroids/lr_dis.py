import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class SegmentCentroidVisualizer(Node):
    def __init__(self):
        super().__init__('segment_centroid_visualizer')

        # `/segments/visualization` 토픽 구독
        self.subscription = self.create_subscription(
            MarkerArray,
            '/segments/visualization',
            self.process_segments,
            10
        )

        # 왼쪽(L) 중심을 연결하는 선을 퍼블리시
        self.left_publisher = self.create_publisher(
            Marker,
            '/left_centroid_path',
            10
        )

        # 오른쪽(R) 중심을 연결하는 선을 퍼블리시
        self.right_publisher = self.create_publisher(
            Marker,
            '/right_centroid_path',
            10
        )

    def process_segments(self, msg):
        left_centroids = []
        right_centroids = []

        for marker in msg.markers:
            if marker.ns == "id":  # 객체 ID 정보 포함된 데이터에서 중심 좌표 추출
                x = marker.pose.position.x
                y = marker.pose.position.y

                # 📌 거리 제한: x 또는 y가 3m 이상이면 무시
                if abs(x) > 2 or abs(y) > 2:
                    continue  

                if x < 0 and y < 0:  # 왼쪽 (-, -)
                    left_centroids.append((x, y))
                elif x < 0 and y > 0:  # 오른쪽 (-, +)
                    right_centroids.append((x, y))

        if len(left_centroids) < 2 and len(right_centroids) < 2:
            self.get_logger().warn("Not enough centroids to create paths.")
            return

        # 왼쪽 선 그리기 (초록색)
        self.publish_marker(self.left_publisher, left_centroids, "left_path", 0.0, 1.0, 0.0)

        # 오른쪽 선 그리기 (파란색)
        self.publish_marker(self.right_publisher, right_centroids, "right_path", 0.0, 0.0, 1.0)

    def publish_marker(self, publisher, points, ns, r, g, b):
        if len(points) < 2:
            return  # 최소한 2개의 점이 있어야 선을 그림

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

        # 중심 좌표를 정렬한 후 선으로 연결
        for x, y in sorted(points, key=lambda p: p[0]):  # x축 기준 정렬
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            marker.points.append(point)

        publisher.publish(marker)
        self.get_logger().info(f"Published {ns} path with {len(points)} points.")

def main(args=None):
    rclpy.init(args=args)
    node = SegmentCentroidVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
