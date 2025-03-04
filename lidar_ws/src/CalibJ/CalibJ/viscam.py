import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import json

from threading import Thread, Lock
import os
from sensor_msgs.msg import LaserScan
from CalibJ.utils.config_loader import load_config, load_json
from CalibJ.module.calibration_module import project_lidar_to_image
from CalibJ.module.clustering_scan import dbscan_clustering

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        self.config = load_config()
        self.get_logger().info(f"Loaded configuration: {self.config}")

        # Load calibration parameters
        self.camera_params = load_json(self.config.cam_calib_result_json)
        self.extrinsic_params = self.load_extrinsic_from_json(self.config.result_path)
        self.get_logger().info("Loaded extrinsic parameters.")

        # Camera setup
        self.capture = cv2.VideoCapture(self.config.camera_number)
        if not self.capture.isOpened():
            self.get_logger().error("Failed to open the camera!")
            raise RuntimeError("Camera initialization failed")

        # ROS 2 Subscriber for LiDAR
        self.scan_sub = self.create_subscription(LaserScan, self.config.scan_topic, self.lidar_callback, 10)

        # Data storage
        self.latest_scan = None
        self.scan_lock = Lock()

        # ROS 2 Timer for visualization loop
        self.create_timer(0.1, self.visualization_loop)

    def load_extrinsic_from_json(self, result_path):
        """Load extrinsic parameters from JSON."""
        extrinsic_file = os.path.join(result_path, "calibration_extrinsic.json")
        if not os.path.exists(extrinsic_file):
            self.get_logger().error(f"Extrinsic file not found: {extrinsic_file}")
            raise FileNotFoundError(f"Extrinsic file not found: {extrinsic_file}")

        with open(extrinsic_file, 'r') as f:
            data = json.load(f)

        rvec = np.array(data["rvec"], dtype=np.float32)
        tvec = np.array(data["tvec"], dtype=np.float32)
        return {"rvec": rvec, "tvec": tvec}

    def lidar_callback(self, msg):
        """Callback to process incoming LiDAR scan data."""
        with self.scan_lock:
            self.latest_scan = msg

    def process_lidar_data(self):
        """Process LiDAR scan data and convert to 3D points."""
        with self.scan_lock:
            if self.latest_scan is None:
                return None

            scan_data = self.latest_scan
            angles = np.linspace(scan_data.angle_min, scan_data.angle_max, len(scan_data.ranges))
            ranges = np.array(scan_data.ranges)

            # Filter invalid range values
            valid = (ranges > scan_data.range_min) & (ranges < scan_data.range_max)
            angles = angles[valid]
            ranges = ranges[valid]

            # Convert polar to Cartesian coordinates
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)
            z = np.zeros_like(x)  # 2D LiDAR data (Z=0)

            points = np.vstack((x, y, z)).T
            return points

    def visualization_loop(self):
        """Main visualization loop."""
        ret, frame = self.capture.read()
        if not ret:
            self.get_logger().error("Failed to capture frame from the camera!")
            return

        lidar_points = self.process_lidar_data()
        if lidar_points is None:
            return

        # Project LiDAR points to the camera frame
        rvec = self.extrinsic_params["rvec"]
        tvec = self.extrinsic_params["tvec"]
        projected_points = project_lidar_to_image(
            lidar_points,
            self.camera_params.camera_matrix,
            self.camera_params.dist_coeffs,
            rvec,
            tvec
        )

        # Visualize the projected points
        for point in projected_points:
            try:
                x, y = map(lambda v: int(round(v)), point)
                if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:
                    cv2.circle(frame, (x, y), 3, (0, 255, 0), -1)  # Green dot
            except ValueError as e:
                self.get_logger().error(f"Failed to draw point {point}: {e}")

        # Display the frame
        cv2.imshow("Visualization", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Shutting down visualization.")
            rclpy.shutdown()

    def destroy_node(self):
        """Cleanup resources."""
        self.capture.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
