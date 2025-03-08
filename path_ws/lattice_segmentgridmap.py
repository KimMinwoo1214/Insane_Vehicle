import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import numpy as np
from std_msgs.msg import Header

class OccupancyGridGenerator(Node):
    def __init__(self):
        super().__init__('occupancy_grid_generator')

        # /segments/visualization 토픽 구독
        self.subscription = self.create_subscription(
            MarkerArray,
            '/segments/visualization',
            self.process_segments,
            10
        )
        
        # 점유 그리드 맵 초기화
        self.grid_width = 100  # 그리드의 가로 크기
        self.grid_height = 100  # 그리드의 세로 크기
        self.resolution = 0.1  # 그리드 해상도 (1격자의 크기)
        
        # 점유 그리드 맵 (0: 비어 있음, 1: 장애물)
        self.occupancy_grid = np.zeros((self.grid_height, self.grid_width), dtype=int)
        
        # 퍼블리셔 설정 (점유 그리드 맵을 퍼블리시)
        self.occupancy_grid_pub = self.create_publisher(MarkerArray, 'occupancy_grid', 10)

    def process_segments(self, msg: MarkerArray):
        """
        세그먼트의 중심 좌표를 점유 그리드에 매핑하여 장애물로 표시
        """
        # 세그먼트 정보 처리
        for marker in msg.markers:
            # 각 세그먼트의 중심 좌표 추출 (marker.pose.position.x, y)
            centroid_x = marker.pose.position.x
            centroid_y = marker.pose.position.y

            # 좌표를 그리드 좌표로 변환
            grid_x = int(centroid_x / self.resolution)
            grid_y = int(centroid_y / self.resolution)

            # 유효한 그리드 좌표인지 확인
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                self.occupancy_grid[grid_y][grid_x] = 1  # 장애물로 표시

        # 점유 그리드 맵을 퍼블리시
        self.publish_occupancy_grid()

        //여기 수정해야 함.
        # 현재 차량 위치를 start로 설정
        start = (current_x, current_y)  # 예: 현재 차량 위치

        # 중앙선 데이터가 존재하는지 확인
        if self.center_lane:
            # 현재 차량 위치에서 가장 가까운 중앙선 점 찾기
            nearest_index = min(range(len(self.center_lane)), key=lambda i: distance.euclidean(start, self.center_lane[i]))

            # 중앙선의 다음 좌표를 goal로 설정
            if nearest_index + 1 < len(self.center_lane):
                goal = self.center_lane[nearest_index + 1]
            else:
                goal = self.center_lane[nearest_index]  # 마지막 좌표면 그냥 현재 좌표 유지
        else:
            goal = start  # 중앙선이 없으면 goal을 start로 유지

        self.get_logger().info(f"Set goal: {goal}")

        # A* 알고리즘으로 경로 찾기
        path = self.a_star(start, goal)

        # 경로 퍼블리시
        self.publish_path(path)

    def publish_occupancy_grid(self):
        """
        점유 그리드 맵을 ROS 메시지로 퍼블리시
        """
        occupancy_grid_msg = MarkerArray()
        
        # 점유 그리드 맵을 MarkerArray 형식으로 변환하여 퍼블리시
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if self.occupancy_grid[y][x] == 1:
                    marker = Marker()
                    marker.header = Header()
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.header.frame_id = 'map'
                    marker.type = marker.SPHERE
                    marker.action = marker.ADD
                    marker.pose.position.x = x * self.resolution
                    marker.pose.position.y = y * self.resolution
                    marker.pose.position.z = 0.0
                    marker.scale.x = self.resolution
                    marker.scale.y = self.resolution
                    marker.scale.z = self.resolution
                    marker.color.a = 1.0
                    marker.color.r = 1.0  # 빨간색으로 장애물 표시

                    occupancy_grid_msg.markers.append(marker)

        # 점유 그리드 맵을 퍼블리시
        self.occupancy_grid_pub.publish(occupancy_grid_msg)
    

    def a_star(self, start, goal):
        """
        A* 알고리즘을 사용하여 경로를 찾는 함수
        """
        # 우선순위 큐를 사용하여 A* 알고리즘 구현
        def heuristic(a, b):
            # 유클리드 거리 (Heuristic 함수)
            return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        def cubic_curve(x_start, y_start, x_end, y_end):
            """
            삼차 함수로 곡선을 생성하는 함수.
            x_start, y_start는 시작점, x_end, y_end는 끝점.
            """
            # 삼차 함수 계수 계산 (y = a3*x^3 + a2*x^2 + a1*x + a0)
            # (시작점과 끝점에 맞춰서 a0, a1, a2, a3를 계산)
            a0 = y_start
            a1 = 0  # 초기 속도는 0으로 설정
            a2 = 3 * (y_end - y_start) / (x_end ** 2)
            a3 = -2 * (y_end - y_start) / (x_end ** 3)

            # 곡선 계산
            x_values = np.linspace(x_start, x_end, 100)
            y_values = a3 * x_values**3 + a2 * x_values**2 + a1 * x_values + a0
            return x_values, y_values


        def neighbors(node):
            """
            인접 노드를 찾을 때, 직선 및 삼차 곡선으로 경로를 생성
            """
            x, y = node
            possible_moves = []

            # 직선 이동 (상하좌우)
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height and self.occupancy_grid[ny][nx] == 0:
                    possible_moves.append((nx, ny))

            # 곡선 이동: 삼차 함수로 경로를 만들어서 그 사이의 점들을 경로로 추가
            # 예를 들어 (x, y) -> (x + 3, y + 3) 같은 경로를 삼차 함수로 만든다고 할 때,
            # 이 두 점 사이에 여러 개의 곡선 점들을 추가할 수 있습니다.
            for dx, dy in [(3, 3), (3, -3), (-3, 3), (-3, -3)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height and self.occupancy_grid[ny][nx] == 0:
                    x_vals, y_vals = cubic_curve(x, y, nx, ny)
                    # 곡선상의 점들을 모두 경로로 추가
                    for i in range(len(x_vals)):
                        possible_moves.append((int(x_vals[i]), int(y_vals[i])))

            return possible_moves

        open_list = []
        closed_list = set()
        came_from = {}

        start_node = start
        goal_node = goal

        heapq.heappush(open_list, (0 + heuristic(start_node, goal_node), 0, start_node))  # (f, g, node)
        
        g_score = {start_node: 0}
        f_score = {start_node: heuristic(start_node, goal_node)}

        while open_list:
            _, g, current = heapq.heappop(open_list)

            if current == goal_node:
                # 목표에 도달하면 경로 반환
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            closed_list.add(current)

            for neighbor in neighbors(current):
                if neighbor in closed_list:
                    continue

                tentative_g_score = g + 1  # 인접한 노드로 가는 비용은 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal_node)
                    heapq.heappush(open_list, (f_score[neighbor], tentative_g_score, neighbor))

        return []  # 경로를 찾을 수 없을 경우 빈 리스트 반환
    
    def publish_path(self, path):
        """
        A* 알고리즘을 통해 찾은 경로를 시각화하여 퍼블리시
        """
        path_msg = Path()

        for (x, y) in path:
            pose = PoseStamped()
            pose.pose.position.x = x * self.resolution
            pose.pose.position.y = y * self.resolution
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        return path_msg
    
def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()