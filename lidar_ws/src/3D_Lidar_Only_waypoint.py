import rclpy
from rclpy.node import Node
from custom_msgs.msg import ObjectInfoArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np
import heapq
import math

class LidarPathPlanner(Node):
    def __init__(self):
        super().__init__('lidar_path_planner')

        self.declare_parameter('map_size', 100)  # meters
        self.declare_parameter('map_resolution', 0.2)  # meters/cell
        self.declare_parameter('robot_radius', 0.5)

        self.map_size = self.get_parameter('map_size').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.robot_radius = self.get_parameter('robot_radius').value

        self.grid_width = int(self.map_size / self.map_resolution)
        self.grid = np.zeros((self.grid_width, self.grid_width), dtype=np.int8)

        self.subscription = self.create_subscription(
            ObjectInfoArray,
            '/lidar_only_object_info',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(Path, '/lidar_only_path', 10)

        self.origin = (self.grid_width // 2, self.grid_width // 2)  # 중심
        self.goal = (self.origin[0] + 20, self.origin[1])  # 임시 목적지

    def callback(self, msg):
        self.grid.fill(0)

        # 장애물 맵 생성
        for obj in msg.object_infos:
            mx = int(obj.x / self.map_resolution) + self.origin[0]
            my = int(obj.y / self.map_resolution) + self.origin[1]
            if 0 <= mx < self.grid_width and 0 <= my < self.grid_width:
                self.mark_obstacle(mx, my)

        path = self.astar(self.origin, self.goal)
        if not path:
            self.get_logger().info("No path found")
            return

        # Path 메시지 생성
        path_msg = Path()
        path_msg.header.frame_id = 'rslidar'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for gx, gy in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = (gx - self.origin[0]) * self.map_resolution
            pose.pose.position.y = (gy - self.origin[1]) * self.map_resolution
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.publisher.publish(path_msg)
        self.get_logger().info(f"Published path with {len(path)} points.")

    def mark_obstacle(self, x, y):
        r = int(self.robot_radius / self.map_resolution)
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_width and 0 <= ny < self.grid_width:
                    self.grid[nx, ny] = 1

    def astar(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start, []))
        visited = set()

        while open_set:
            f, cost, current, path = heapq.heappop(open_set)
            if current in visited:
                continue
            visited.add(current)

            if current == goal:
                return path + [current]

            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nx, ny = current[0] + dx, current[1] + dy
                if not (0 <= nx < self.grid_width and 0 <= ny < self.grid_width):
                    continue
                if self.grid[nx, ny] == 1:
                    continue
                heapq.heappush(open_set, (
                    cost + 1 + self.heuristic((nx, ny), goal),
                    cost + 1,
                    (nx, ny),
                    path + [current]
                ))

        return None

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])


def main(args=None):
    rclpy.init(args=args)
    node = LidarPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

