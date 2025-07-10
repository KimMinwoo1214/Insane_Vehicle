import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import ObjectInfoArray
import numpy as np
import heapq
import math
import tf_transformations
from std_msgs.msg import Header


class SmoothCurvePlanner(Node):
    def __init__(self):
        super().__init__('smooth_curve_planner')

        self.map_size = 100
        self.map = np.zeros((self.map_size, self.map_size), dtype=np.int8)
        self.robot_radius = 3
        self.origin = (50, 50)

        self.subscription = self.create_subscription(
            ObjectInfoArray,
            '/lidar_only_object_info',
            self.callback,
            10)

        self.path_pub = self.create_publisher(Path, '/lidar_only_path', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/lidar_obstacle_map', 10)

    def callback(self, msg):
        self.map[:] = 0

        for obj in msg.object_infos:
            ox = int(obj.x / 10.0 * self.map_size)
            oy = int(obj.y / 10.0 * self.map_size)
            for dx in range(-self.robot_radius, self.robot_radius + 1):
                for dy in range(-self.robot_radius, self.robot_radius + 1):
                    nx, ny = ox + dx, oy + dy
                    if 0 <= nx < self.map_size and 0 <= ny < self.map_size:
                        self.map[ny, nx] = 100

        goal = self.find_dynamic_goal()
        path = self.smooth_a_star(self.origin, goal)

        if path:
            path_msg = Path()
            path_msg.header.frame_id = "base_link"
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for (x, y) in path:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = (x - self.origin[0]) / 10.0
                pose.pose.position.y = (y - self.origin[1]) / 10.0
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)

        self.publish_occupancy_grid()

    def find_dynamic_goal(self):
        for y in range(self.map_size - 1, self.origin[1], -1):
            for x in range(self.map_size):
                if self.map[y, x] == 0:
                    return (x, y)
        return self.origin

    def smooth_a_star(self, start, goal):
        if self.map[goal[1], goal[0]] != 0:
            return None

        open_set = []
        heapq.heappush(open_set, (self.heuristic(start, goal), 0, start, [start]))
        visited = set()

        directions = [(1, 0), (0, 1), (-1, 0), (0, -1),
                      (1, 1), (-1, 1), (-1, -1), (1, -1)]

        while open_set:
            _, cost, current, path = heapq.heappop(open_set)

            if current == goal:
                return self.smooth_path(path)

            if current in visited:
                continue
            visited.add(current)

            for dx, dy in directions:
                nx, ny = current[0] + dx, current[1] + dy
                if 0 <= nx < self.map_size and 0 <= ny < self.map_size:
                    if self.map[ny, nx] == 0:
                        new_cost = cost + math.hypot(dx, dy)
                        heapq.heappush(open_set, (
                            new_cost + self.heuristic((nx, ny), goal),
                            new_cost,
                            (nx, ny),
                            path + [(nx, ny)]
                        ))
        return None

    def smooth_path(self, path):
        # 간단한 보간: 중간 점 보간 (더 정교한 cubic도 가능)
        if len(path) < 3:
            return path
        smooth = [path[0]]
        for i in range(1, len(path) - 1):
            mid_x = (path[i][0] + path[i+1][0]) / 2
            mid_y = (path[i][1] + path[i+1][1]) / 2
            smooth.append((mid_x, mid_y))
            smooth.append(path[i+1])
        return smooth

    def heuristic(self, a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def publish_occupancy_grid(self):
        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "base_link"

        grid.info.resolution = 0.1
        grid.info.width = self.map_size
        grid.info.height = self.map_size
        grid.info.origin.position.x = -self.map_size / 2 * 0.1
        grid.info.origin.position.y = -self.map_size / 2 * 0.1
        grid.info.origin.position.z = 0.0
        quat = tf_transformations.quaternion_from_euler(0, 0, 0)
        grid.info.origin.orientation.x = quat[0]
        grid.info.origin.orientation.y = quat[1]
        grid.info.origin.orientation.z = quat[2]
        grid.info.origin.orientation.w = quat[3]

        grid.data = self.map.flatten().tolist()
        self.map_pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = SmoothCurvePlanner()
    rclpy.spin(node)
    rclpy.shutdown()

