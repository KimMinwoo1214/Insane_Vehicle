import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
from scipy.spatial import distance

class RoadLaneVisualizer(Node):
    def __init__(self):
        super().__init__('road_lane_visualizer')

        # `/segments/visualization` 토픽 구독
        self.subscription = self.create_subscription(
            MarkerArray,
            '/segments/visualization',
            self.process_segments,
            10
        )

        # 왼쪽(L) 차선을 위한 퍼블리셔
        self.left_lane_publisher = self.create_publisher(
            Marker,
            '/left_lane',
            10
        )

        # 오른쪽(R) 차선을 위한 퍼블리셔
        self.right_lane_publisher = self.create_publisher(
            Marker,
            '/right_lane',
            10
        )

        # 중앙 차선을 위한 퍼블리셔
        self.center_lane_publisher = self.create_publisher(
            Marker,
            '/center_lane',
            10
        )

    def process_segments(self, msg):
        left_lane = []
        right_lane = []

        # 도로 중심을 찾기 위한 변수
        all_y_values = []

        for marker in msg.markers:
            if marker.ns == "id":  # 객체 중심 좌표만 가져옴
                x = marker.pose.position.x
                y = marker.pose.position.y

                # 📌 x < 0 인 객체 무시
                if x > 0:
                    continue

                # 📌 거리 제한: x 또는 y가 3m 이상이면 무시
                if abs(x) > 6 or abs(y) > 6:
                    continue  

                all_y_values.append(y)

        if not all_y_values:
            self.get_logger().warn("No valid objects detected.")
            return
        
        # 📌 도로 중심선 찾기 (평균 Y 값 사용)
        road_center_y = sum(all_y_values) / len(all_y_values)

        # 다시 객체를 분류
        for marker in msg.markers:
            if marker.ns == "id":
                x = marker.pose.position.x
                y = marker.pose.position.y

                # 📌 x < 0 인 객체 무시
                if x > 0:
                    continue

                if abs(x) > 6 or abs(y) > 6:
                    continue  

                # 📌 도로 중심을 기준으로 좌우 차선 분류
                if y < road_center_y:  # 왼쪽 차선
                    left_lane.append((x, y))
                else:  # 오른쪽 차선
                    right_lane.append((x, y))

        if len(left_lane) < 2 and len(right_lane) < 2:
            self.get_logger().warn("Not enough centroids to create lanes.")
            return

        # 📌 가까운 순서대로 정렬 후 차선 생성
        left_lane = self.sort_by_distance(left_lane)
        right_lane = self.sort_by_distance(right_lane)

        # 📌 중앙 차선 계산
        center_lane = self.calculate_center_lane(left_lane, right_lane)

        # 📌 좌우 차선을 곡선으로 연결
        self.publish_marker(self.left_lane_publisher, left_lane, "left_lane", 0.0, 1.0, 0.0)  # 초록색
        self.publish_marker(self.right_lane_publisher, right_lane, "right_lane", 0.0, 0.0, 1.0)  # 파란색
        self.publish_marker(self.center_lane_publisher, center_lane, "center_lane", 1.0, 1.0, 1.0)  # 흰색

    def sort_by_distance(self, points):
        """거리가 가까운 순서대로 정렬"""
        if len(points) < 2:
            return points  # 정렬할 필요 없음

        points = np.array(points)
        sorted_points = [points[0]]  # 시작점
        remaining_points = points[1:].tolist()

        while remaining_points:
            last_point = sorted_points[-1]
            nearest_point = min(remaining_points, key=lambda p: distance.euclidean(last_point, p))
            sorted_points.append(nearest_point)
            remaining_points.remove(nearest_point)

        return sorted_points

    def calculate_center_lane(self, left_lane, right_lane):
        """ 왼쪽과 오른쪽 차선의 중심을 계산하여 중앙 차선을 생성 """
        center_lane = []

        min_length = min(len(left_lane), len(right_lane))

        for i in range(min_length):
            left_x, left_y = left_lane[i]
            right_x, right_y = right_lane[i]

            center_x = (left_x + right_x) / 2
            center_y = (left_y + right_y) / 2

            center_lane.append((center_x, center_y))

        return center_lane

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

        # 중심 좌표를 거리 순서대로 정렬하여 선을 부드럽게 연결
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
    node = RoadLaneVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

