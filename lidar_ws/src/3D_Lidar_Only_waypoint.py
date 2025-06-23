import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class WaypointGenerator(Node):
    def __init__(self):
        super().__init__('lidar_waypoint_generator')

        self.declare_parameter('max_distance', 20.0)
        self.max_distance = self.get_parameter('max_distance').get_parameter_value().double_value

        self.subscription = self.create_subscription(
            PoseArray,
            '/detected_cones',
            self.listener_callback,
            10)

        self.publisher_ = self.create_publisher(Path, '/center_path', 10)

    def listener_callback(self, msg):
        cones = msg.poses
        cones = [pose for pose in cones if math.sqrt(pose.position.x ** 2 + pose.position.y ** 2) <= self.max_distance]
        if len(cones) < 2:
            return

        cones.sort(key=lambda pose: pose.position.x)

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'rslidar'

        for i in range(len(cones) - 1):
            mid_pose = PoseStamped()
            mid_pose.header = path_msg.header
            mid_pose.pose.position.x = (cones[i].position.x + cones[i+1].position.x) / 2.0
            mid_pose.pose.position.y = (cones[i].position.y + cones[i+1].position.y) / 2.0
            mid_pose.pose.position.z = 0.0
            path_msg.poses.append(mid_pose)

        self.publisher_.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

