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

        # 중앙 선 퍼블리시
        self.center_publisher = self.create_publisher(
            Marker,
            '/center_path',
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

        # 중앙 선 그리기 (빨간색)
        self.create_center_line(left_centroids, right_centroids)
       
    def create_center_line(self, left_centroids, right_centroids):
        # 왼쪽 선과 오른쪽 선의 기울기 계산
        m_left, b_left = self.fit_line(left_centroids)
        m_right, b_right = self.fit_line(right_centroids)

        # 중앙선의 기울기와 절편을 평균으로 구함
        m_center = (m_left + m_right) / 2
        b_center = (b_left + b_right) / 2

        # 중앙 선 좌표 생성 (x 범위에 맞게 중앙선 점 계산)
        center_points = self.generate_center_line(min(x for x, y in left_centroids), max(x for x, y in right_centroids), m_center, b_center)

        # 중앙 선 퍼블리시
        self.publish_marker(self.center_publisher, center_points, "center_path", 1.0, 0.0, 0.0)

    def fit_line(self, points):
        """ 주어진 (x, y) 좌표 리스트를 직선 방정식 y = mx + b로 피팅 """
        x_vals = np.array([p[0] for p in points])
        y_vals = np.array([p[1] for p in points])
        
        # 직선 피팅 (최소자승법)
        m, b = np.polyfit(x_vals, y_vals, 1)
        
        return m, b  # 기울기(m)와 절편(b)

    def generate_center_line(self, x_start, x_end, m, b, num_points=50):
        """ x_start에서 x_end까지 중앙 선의 좌표를 생성 """
        x_vals = np.linspace(x_start, x_end, num_points)
        y_vals = m * x_vals + b
        return list(zip(x_vals, y_vals))

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