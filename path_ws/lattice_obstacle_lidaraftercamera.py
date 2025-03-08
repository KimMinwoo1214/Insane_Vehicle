import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import heapq


class LatticePlanner(Node):
    def __init__(self):
        super().__init__('lattice_planner')

        # /object_info 토픽 구독 (카메라 기반 객체 정보)
        self.subscription = self.create_subscription(
            String,
            '/object_info',
            self.process_object_info,
            10
        )
        
        # 점유 그리드 맵 초기화
        self.grid_width = 100  # 그리드의 가로 크기
        self.grid_height = 100  # 그리드의 세로 크기
        self.resolution = 0.1  # 그리드 해상도 (1격자의 크기)
        ##이 값은 테스트 하며 조절하기.
        
        # 점유 그리드 맵 (0: 비어 있음, 1: 장애물)
        self.occupancy_grid = np.zeros((self.grid_height, self.grid_width), dtype=int)
        
        # 퍼블리셔 설정 (점유 그리드 맵을 퍼블리시)
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.occupancy_grid_pub = self.create_publisher(MarkerArray, 'occupancy_grid', 10)

    def process_object_info(self, msg: String):
        """
        /object_info 토픽에서 받은 데이터를 점유 그리드 맵에 반영
        """
        try:
            data = msg.data.split(',')
            if data[0] == 'obstacle':
                x = float(data[1])
                y = float(data[2])
                distance = float(data[3])

                # 좌표를 그리드 좌표로 변환
                grid_x = int(x / self.resolution)
                grid_y = int(y / self.resolution)

                # 유효한 그리드 좌표인지 확인 후 장애물 표시
                if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                    self.occupancy_grid[grid_y][grid_x] = 1
        
        except Exception as e:
            self.get_logger().error(f"Failed to process object info: {e}")

        # 점유 그리드 퍼블리시
        self.publish_occupancy_grid()

    def publish_occupancy_grid(self):
        """
        점유 그리드 맵을 ROS 메시지로 퍼블리시
        """
        occupancy_grid_msg = MarkerArray()
        
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if self.occupancy_grid[y][x] == 1:
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.header.stamp = self.get_clock().now().to_msg()
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

        self.occupancy_grid_pub.publish(occupancy_grid_msg)

    def a_star(self, start, goal):
        """
        A* 알고리즘을 사용하여 경로를 찾는 함수
        """
        ##경로 만드는 부분 수정하기

        def heuristic(a, b):
            # 유클리드 거리 (Heuristic 함수)
            return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        def cubic_curve(x_start, y_start, x_end, y_end):
            """
            Generates a cubic curve between (x_start, y_start) and (x_end, y_end).
            """
            t = np.linspace(0, 1, 100)  # Parameter t from 0 to 1 for interpolation
            x_vals = (1 - t) * x_start + t * x_end
            y_vals = (1 - t) * y_start + t * y_end  # Linear interpolation for simplicity
            return x_vals, y_vals
        
        def neighbors(self, node):
            """
            Find neighboring nodes, including straight and cubic curve paths.
            """
            x, y = node
            possible_moves = []

            # Straight moves (up, down, left, right)
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height and self.occupancy_grid[ny][nx] == 0:
                    possible_moves.append((nx, ny))

            # Cubic curve moves: Check if the path between (x, y) and (nx, ny) is valid
            for dx, dy in [(3, 3), (3, -3), (-3, 3), (-3, -3)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height and self.occupancy_grid[ny][nx] == 0:
                    # Get cubic curve points
                    x_vals, y_vals = cubic_curve(x, y, nx, ny)

                    # Check if all points along the curve are free of obstacles
                    is_valid = True
                    for i in range(len(x_vals)):
                        cur_x, cur_y = int(x_vals[i]), int(y_vals[i])
                        if not (0 <= cur_x < self.grid_width and 0 <= cur_y < self.grid_height and self.occupancy_grid[cur_y][cur_x] == 0):
                            is_valid = False
                            break
                    if is_valid:
                        for i in range(len(x_vals)):
                            possible_moves.append((int(x_vals[i]), int(y_vals[i])))

            return possible_moves

        open_list = []
        closed_list = set()
        came_from = {}

        start_node = start
        goal_node = goal
        ##목표점 뭘로 설정할건지

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
        if not path:
            self.get_logger().warn("No path found!")
            return
        ## 이 부분 수정해야 함. 경로가 없는 게 아니라 래티스 모드일 때로

        path_msg = Path()

        for (x, y) in path:
            pose = PoseStamped()
            pose.pose.position.x = x * self.resolution
            pose.pose.position.y = y * self.resolution
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LatticePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()