import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class LanePlanner(Node):
    def __init__(self):
        super().__init__('lane_planner_node')

        # ===== Parameters =====
        self.img_width = 640
        self.img_height = 480
        self.latest_img = None
        self.bridge = CvBridge()

        # ===== ROS QoS =====
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.sub_polygon = self.create_subscription(
            PolygonStamped,
            '/yolo_polygon',
            self.polygon_callback,
            qos
        )
        self.sub_img = self.create_subscription(
            Image,
            '/video_frames',
            self.image_callback,
            qos
        )
        self.pub_angle = self.create_publisher(Float32, '/lane_steering_angle', qos)

    def image_callback(self, msg):
        try:
            self.latest_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def polygon_callback(self, msg):
        if self.latest_img is None:
            self.get_logger().warn("⚠️ No image received yet, skip.")
            return

        if len(msg.polygon.points) < 3:
            self.get_logger().warn("⚠️ Polygon too small, skip.")
            return

        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.int32)

        # Create image for visualization
        overlay_img = self.latest_img.copy()
        cv2.polylines(overlay_img, [pts], isClosed=True, color=(255, 255, 0), thickness=2)

        # Draw left lane line (assuming pts[0] and pts[1] define it)
        if len(pts) >= 2:
            cv2.line(overlay_img, tuple(pts[0]), tuple(pts[1]), (0, 255, 255), 2) # Cyan

        # Draw right lane line (assuming pts[3] and pts[2] define it)
        if len(pts) >= 4:
            cv2.line(overlay_img, tuple(pts[3]), tuple(pts[2]), (255, 0, 255), 2) # Magenta

        # Define the middle line for intersection
        middle_line = ((self.img_width // 2, self.img_height), (self.img_width // 2, 0))
        cv2.line(overlay_img, middle_line[0], middle_line[1], (128, 128, 128), 1)

        last_intersection, intersected_segment = self.find_last_intersection_and_tangent(pts, middle_line)

        if last_intersection and intersected_segment:
            # Draw intersection and segment
            cv2.circle(overlay_img, (int(last_intersection[0]), int(last_intersection[1])), 5, (0, 0, 255), -1)
            cv2.line(overlay_img, intersected_segment[0], intersected_segment[1], (0, 255, 0), 2)

            # Calculate steering angle
            tangent_angle = self.calculate_tangent_angle(intersected_segment)

            # Publish the angle
            angle_msg = Float32()
            angle_msg.data = float(tangent_angle)
            self.pub_angle.publish(angle_msg)
            self.get_logger().info(f"✅ Angle: {tangent_angle:.2f}, Publishing...")

            # Draw the final angle
            start_point = (self.img_width // 2, self.img_height)
            length = 50
            # Angle is in degrees, 0 is straight. Convert to radians for drawing.
            angle_rad = np.deg2rad(tangent_angle + 90) # Add 90 to align with image coordinates
            end_point = (
                int(start_point[0] + length * np.cos(angle_rad)),
                int(start_point[1] - length * np.sin(angle_rad))
            )
            cv2.line(overlay_img, start_point, end_point, (0, 0, 255), 3)
        else:
            self.get_logger().warn("⚠️ No valid intersections found.")

        cv2.imshow("Lane Planner", overlay_img)
        cv2.waitKey(1)

    def find_last_intersection_and_tangent(self, polygon_pts, line):
        intersections = []
        for i in range(len(polygon_pts)):
            p1 = polygon_pts[i]
            p2 = polygon_pts[(i + 1) % len(polygon_pts)]
            intersection_point = self.line_intersection((p1, p2), line)
            if intersection_point:
                intersections.append({'point': intersection_point, 'segment': (p1, p2)})

        if not intersections:
            return None, None

        # Find the intersection with the highest y-value (furthest away)
        last_intersection_data = max(intersections, key=lambda item: item['point'][1])

        return last_intersection_data['point'], last_intersection_data['segment']

    def line_intersection(self, line1, line2):
        p1, p2 = line1
        p3, p4 = line2

        x1, y1 = p1.astype(float)
        x2, y2 = p2.astype(float)
        x3, y3 = p3
        x4, y4 = p4

        den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(den) < 1e-6:
            return None  # Parallel

        t_num = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)
        u_num = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3))

        t = t_num / den
        u = u_num / den

        if 0 <= t <= 1 and 0 <= u <= 1:
            ix = x1 + t * (x2 - x1)
            iy = y1 + t * (y2 - y1)
            return (ix, iy)
        return None

    def calculate_tangent_angle(self, segment):
        p1, p2 = segment
        # Calculate the angle relative to the positive y-axis (upwards)
        # This will give 0 for a vertical line, positive for leaning right, negative for leaning left
        angle_rad = np.arctan2(p2[0] - p1[0], -(p2[1] - p1[1]))
        angle_deg = np.rad2deg(angle_rad)
        return angle_deg


def main(args=None):
    rclpy.init(args=args)
    node = LanePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

