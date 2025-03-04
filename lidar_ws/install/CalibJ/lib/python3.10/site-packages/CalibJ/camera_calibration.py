import cv2
import json
import os
import numpy as np
import rclpy
from rclpy.node import Node
from CalibJ.utils.config_loader import load_config

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        self.config = load_config()
        self.get_logger().info(f"Loaded configuration: {self.config}")
        self.camera_number = self.config.camera_number
        self.output_dir = self.config.result_path

        self.image_save_dir = os.path.join(self.output_dir, "camera_calibration")
        os.makedirs(self.image_save_dir, exist_ok=True)  # Ensure the directory exists

        self.pattern_size = (9, 6)  # Checkerboard pattern size (9x6 grid)
        self.square_size = 0.036  # Checkerboard square size in meters
        self.object_points = []  # 3D points in the real world
        self.image_points = []  # 2D points in image plane
        self.calibration_images = []  # Images used for calibration

        self.camera = cv2.VideoCapture(self.camera_number)
        if not self.camera.isOpened():
            self.get_logger().error("Could not open camera!")
            raise RuntimeError("Camera initialization failed")

        self.init_checkerboard_points()

   

    def init_checkerboard_points(self):
        # Generate 3D points for checkerboard
        objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        self.object_points_template = objp

    def capture_images(self):
        self.get_logger().info("Starting image capture for calibration. Press 'q' to quit, 'c' to capture.")
        image_count = 0

        while True:
            ret, frame = self.camera.read()
            if not ret:
                self.get_logger().error("Failed to capture frame")
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)

            if ret:
                self.get_logger().info("Checkerboard detected, press 'c' to capture or 'q' to quit.")
                cv2.drawChessboardCorners(frame, self.pattern_size, corners, ret)

            cv2.imshow("Calibration", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c') and ret:
                self.get_logger().info("Capturing image with detected checkerboard.")
                self.image_points.append(corners)
                self.object_points.append(self.object_points_template)
                self.calibration_images.append(frame)

                # Save the captured image
                image_path = os.path.join(self.image_save_dir, f"image_{image_count:03d}.png")
                cv2.imwrite(image_path, frame)
                self.get_logger().info(f"Image saved to {image_path}")
                image_count += 1

            elif key == ord('q'):
                self.get_logger().info("Exiting image capture mode and starting calibration.")
                break

        cv2.destroyAllWindows()

    def calibrate_camera(self):
        if len(self.image_points) < 5:
            self.get_logger().error("Insufficient images for calibration. Capture at least 5 images.")
            return False

        self.get_logger().info("Calibrating camera...")
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.object_points, self.image_points, self.calibration_images[0].shape[1::-1], None, None
        )

        if not ret:
            self.get_logger().error("Calibration failed!")
            return False

        self.save_calibration(camera_matrix, dist_coeffs)
        self.get_logger().info("Calibration successful. Results saved.")
        return True

    def save_calibration(self, camera_matrix, dist_coeffs):
        calibration_data = {
            "camera_matrix": camera_matrix.tolist(),
            "dist_coeffs": dist_coeffs.tolist(),
        }

        output_file = os.path.join(self.output_dir, "calibration_result.json")
        with open(output_file, 'w') as f:
            json.dump(calibration_data, f, indent=4)
        self.get_logger().info(f"Calibration data saved to {output_file}")

    def destroy_node(self):
        self.camera.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    try:
        node.capture_images()
        node.calibrate_camera()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
