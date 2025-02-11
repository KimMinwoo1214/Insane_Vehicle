import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np

class SegmentCentroidCalculator(Node):
    def __init__(self):
        super().__init__('segment_centroid_calculator')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/segments',
            self.process_segments,
            10)
        self.subscription  # prevent unused variable warning

    def process_segments(self, msg):
        for segment in msg.segments:
            # 객체의 ID 출력
            segment_id = segment.id
            points = segment.points

            # 객체 중심 좌표 (centroid) 계산
            if len(points) > 0:
                x_vals = [p.x for p in points]
                y_vals = [p.y for p in points]
                centroid_x = np.mean(x_vals)
                centroid_y = np.mean(y_vals)

                self.get_logger().info(f"Segment {segment_id}: Centroid = ({centroid_x}, {centroid_y})")

def main(args=None):
    rclpy.init(args=args)
    node = SegmentCentroidCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
