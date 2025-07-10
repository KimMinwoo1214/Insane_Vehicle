#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import PolygonStamped
import numpy as np
import cv2

class HybridTargetSliceAngleEstimator(Node):
    def __init__(self):
        super().__init__('hybrid_target_slice_angle_node')

        # ===== Configuration Parameters =====
        # Grouping parameters in a dictionary makes them easier to manage and tune.
        self.config = {
            'img_w': 640,
            'img_h': 480,
            'roi_ymin': 300,
            'roi_ymax': 400,
            'slice_step': 10,  # Pixel height of each horizontal slice
            'visualize': True, # Flag to enable/disable visualization
        }
        # The reference point is kept separate as it's a derived calculation.
        self.reference_point = np.array([self.config['img_w'] / 2.0, self.config['img_h']])

        # ===== ROS2 QoS and Pub/Sub Setup =====
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.sub = self.create_subscription(
            PolygonStamped,
            '/yolo_polygon',
            self.callback,
            qos
        )
        self.pub = self.create_publisher(Float32, '/lane_steering_angle', qos)

        self.get_logger().info("✅ HybridTargetSliceAngleEstimator has started.")

    def callback(self, msg):
        if len(msg.polygon.points) < 3:
            self.pub.publish(Float32(data=0.0))
            return

        # Polygon → contour mask generation
        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.int32)
        mask = np.zeros((self.config['img_h'], self.config['img_w']), dtype=np.uint8)
        cv2.polylines(mask, [pts.reshape(-1, 1, 2)], isClosed=True, color=255, thickness=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            self.pub.publish(Float32(data=0.0))
            return

        # Use the largest contour as the lane
        contour = max(contours, key=cv2.contourArea)
        contour_pts = contour.reshape(-1, 2)

        # === Efficient Target Point Calculation ===
        # Group x-coordinates by their y-slice in a single pass for efficiency.
        sliced_lanes = {}
        for x, y in contour_pts:
            if self.config['roi_ymin'] <= y <= self.config['roi_ymax']:
                slice_idx = (y - self.config['roi_ymin']) // self.config['slice_step']
                if slice_idx not in sliced_lanes:
                    sliced_lanes[slice_idx] = []
                sliced_lanes[slice_idx].append(x)
        
        target_pts = []
        for slice_idx in sorted(sliced_lanes.keys()):
            center_x = np.mean(sliced_lanes[slice_idx])
            # Calculate the y-center of the slice for the target point
            center_y = self.config['roi_ymin'] + slice_idx * self.config['slice_step'] + self.config['slice_step'] / 2
            target_pts.append([center_x, center_y])
        
        target_pts = np.array(target_pts)

        if len(target_pts) < 2:
            self.pub.publish(Float32(data=0.0))
            return

        # === Reference-based Angle Calculation ===
        # A single, clear calculation for the steering angle.
        angles = []
        for pt in target_pts:
            dx = pt[0] - self.reference_point[0]
            dy = self.reference_point[1] - pt[1]
            if dy == 0:
                continue
            angle_rad = np.arctan2(dx, dy)
            angles.append(90.0 + np.rad2deg(angle_rad))
        
        final_angle = np.mean(angles) if len(angles) > 0 else 90.0

        self.pub.publish(Float32(data=final_angle))
        self.get_logger().info(f"✅ Calculated angle: {final_angle:.2f}°")

        # === Robust Visualization ===
        if self.config['visualize']:
            try:
                vis = np.zeros((self.config['img_h'], self.config['img_w'], 3), dtype=np.uint8)
                for pt in target_pts:
                    pt_int = tuple(np.round(pt).astype(int))
                    cv2.circle(vis, pt_int, 3, (0, 255, 0), -1)
                    cv2.line(vis, tuple(self.reference_point.astype(int)), pt_int, (255, 255, 255), 1)
                
                cv2.circle(vis, tuple(self.reference_point.astype(int)), 4, (0, 0, 255), -1)
                cv2.putText(vis, f"{final_angle:.2f} deg", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                
                cv2.imshow("Lane Center Target", vis)
                cv2.waitKey(1)
            except cv2.error:
                self.get_logger().warn("Could not display visualization. (No display environment?)")
                # Disable visualization for future frames to avoid repeated warnings.
                self.config['visualize'] = False

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = HybridTargetSliceAngleEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
