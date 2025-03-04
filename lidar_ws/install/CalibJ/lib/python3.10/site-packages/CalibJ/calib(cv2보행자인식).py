#######
# KJY #
#######

import rclpy
from rclpy.node import Node
import os
from sensor_msgs.msg import LaserScan, PointCloud2
import cv2  
from threading import Thread, Lock
import time  
import numpy as np
from datetime import datetime
from aprilgrid import Detector
from CalibJ.utils.config_loader import load_config, load_json, load_extrinsic_from_json, save_extrinsic_to_json
from CalibJ.utils.clustering_vis import points_to_pointcloud2
from CalibJ.module.clustering_scan import save_execution_statistics, dbscan_clustering, display_clusters, world_to_pixel # , optics_clustering
from CalibJ.module.calibration_module import calibration_2dlidar_camera, process_combined_coordinates, project_lidar_to_image, convert_image_to_lidar
from CalibJ.module.apriltag_detect import detect_apriltag
from CalibJ.evaluate.clustering_eval import evaluate_clustering, record_evaluation_result 
from CalibJ.module.tracking import calculate_cluster_centers, ClusterTracker
from CalibJ.module.abs_distance_module import show_pixel_spectrum, filter_noise_by_histogram
from CalibJ.evaluate.calibration_eval import calculate_reprojection_error_2d, save_reprojection_errors_to_csv

from cv2 import HOGDescriptor

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.config = load_config()
        self.get_logger().info(f"Loaded configuration: {self.config}")

        self.camera_params = load_json(self.config.cam_calib_result_json)


        # OpenCV HOG 기반 사람 보행자 검출 초기화
        self.hog = HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())



        # ROS 2 Subscribers and Publishers
        self.scan_sub = self.create_subscription(LaserScan, self.config.scan_topic, self.scan_callback, 10)
        self.cluster_pub = self.create_publisher(PointCloud2, self.config.cluster_topic, 10)

        # Data Management
        self.latest_scan = None
        self.latest_camera_frame = None
        self.scan_lock = Lock()
        self.running = True
        self.lidar_features = []

        # AprilTag Detector and Tracker
        self.detector = Detector("t36h11")
        self.apriltag_features = []
        self.cluster_tracker = ClusterTracker()

        self.vis_distance = self.config.vis_distance 

        self.pix_cluster_points = None

        # Mouse Interaction
        self.click_x = None
        self.click_y = None
        cv2.namedWindow("Camera and Clusters")
        cv2.setMouseCallback("Camera and Clusters", self.on_mouse_click)

        # Camera Initialization
        self.capture = cv2.VideoCapture(self.config.camera_number)
        if not self.capture.isOpened():
            self.get_logger().error("Failed to open the camera!")
            raise RuntimeError("Camera initialization failed")

        # ROS 2 Timers for main thread processing
        self.create_timer(0.1, self.camera_loop)  # 100ms interval
        self.create_timer(0.1, self.process_data_loop)  # 100ms interval
        self.create_timer(0.1, self.show_visualization)  # 100ms interval for GUI

        self.get_logger().info("All modules are ready.")

    def scan_callback(self, msg):
        """Callback for LaserScan topic."""
        with self.scan_lock:
            self.latest_scan = {'timestamp': time.time(), 'data': msg}

    def camera_loop(self):
        """Periodic camera frame capture."""
        ret, frame = self.capture.read()
        if ret:
            self.latest_camera_frame = {'timestamp': time.time(), 'frame': frame}
        else:
            self.get_logger().error("Failed to capture frame from the camera!")

    def process_data_loop(self):
        """Synchronize and process data."""
        with self.scan_lock:
            if self.latest_scan and self.latest_camera_frame:
                scan_time = self.latest_scan['timestamp']
                camera_time = self.latest_camera_frame['timestamp']

                if abs(scan_time - camera_time) < 0.3:  # Synchronization threshold
                    scan_data = self.latest_scan['data']
                    camera_frame = self.latest_camera_frame['frame']
                    self.process_synchronized_data(scan_data, camera_frame)

    def process_synchronized_data(self, scan_data, camera_frame):
        """Process synchronized scan and camera data."""
        labels, cluster_points, distances, _ = dbscan_clustering(
            scan_data, epsilon=self.config.epsilon, min_samples=self.config.min_samples
        )

        self.distances = distances

        # rviz 시각화용
        # cluster_msg = points_to_pointcloud2(labels, cluster_points, frame_id=self.config.cluster_frame)
        # self.cluster_pub.publish(cluster_msg)

        pix_labels, pix_cluster_points = world_to_pixel(labels, cluster_points, max_distance=self.config.max_distance)
 
        self.pix_cluster_points = pix_cluster_points

        # update cluster center positions
        self.cluster_tracker.update_clusters(pix_labels, pix_cluster_points)


        # 마우스 클릭으로 중심 좌표 추적 시작
        if self.click_x is not None and self.click_y is not None:
            selected_center = self.cluster_tracker.select_tracked_center(self.click_x, self.click_y, selection_radius=100)
            if selected_center:
                pass
                # self.get_logger().info(f"Tracking cluster center at: {selected_center}")
            self.click_x = None
            self.click_y = None

        # 칼만 필터 추적
        if self.cluster_tracker.is_tracking:
            predicted_position = self.cluster_tracker.track()
            if predicted_position is not None:
                pass
                # self.get_logger().info(f"Tracking predicted position: {predicted_position}")


        # Visualization preparation
        self.cluster_canvas = display_clusters(
            pix_labels,
            pix_cluster_points,
            canvas_size=self.config.canvas_size,
            cluster_tracker=self.cluster_tracker,
            only_clustering=False
        )

        self.camera_frame = camera_frame

    def calculate_average_distance_in_bbox(self, x, y, w, h):
        """
        Calculate the average distance of LiDAR points within the bounding box.

        Args:
            x (int): Top-left x-coordinate of the bounding box.
            y (int): Top-left y-coordinate of the bounding box.
            w (int): Width of the bounding box.
            h (int): Height of the bounding box.

        Returns:
            float: Average distance of points within the bounding box, or None if no points are found.
        """
        distances_in_bbox = []

        for (px, py), distance in zip(self.filtered_points, self.filtered_distances):
            if x <= px <= x + w and y <= py <= y + h:
                distances_in_bbox.append(distance)

        if distances_in_bbox:
            return np.mean(distances_in_bbox)
        else:
            return None
        
    def show_visualization(self):
        """Main thread OpenCV GUI rendering."""
        if hasattr(self, 'cluster_canvas') and self.camera_frame is not None:
            try:
                detect_frame = self.camera_frame.copy()

                # LiDAR 데이터를 카메라 프레임에 투영
                if hasattr(self, 'rvec') and len(self.pix_cluster_points) > 0:
                    lidar_points = self.pix_cluster_points

                    # 투영된 점 계산
                    projected_points = project_lidar_to_image(
                        lidar_points,
                        self.camera_params.camera_matrix,
                        self.camera_params.dist_coeffs,
                        self.rvec,
                        self.tvec
                    )

                    # 노이즈 필터링
                    filtered_points, filtered_distances = filter_noise_by_histogram(
                        projected_points, 
                        self.distances,
                        frame_height=self.camera_frame.shape[0],
                        min_frequency=self.config.min_frequency,
                        num_bins=self.config.num_bins
                    )

                    self.filtered_points = filtered_points
                    self.filtered_distances = filtered_distances

                    bounding_boxes, _ = self.hog.detectMultiScale(detect_frame, winStride=(8, 8), padding=(8, 8), scale=1.05)

                    # 보행자 검출된 바운딩 박스 표시
                    for (x, y, w, h) in bounding_boxes:
                        cv2.rectangle(detect_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                        # 평균 거리 계산
                        average_distance = self.calculate_average_distance_in_bbox(x, y, w, h)
                        if average_distance is not None:
                            # 텍스트 위치: 바운딩 박스 내부 좌하단 (x, y + h - 5)
                            text_x, text_y = x + 5, y + h - 5
                            text = f"{average_distance:.2f} m"
                            cv2.putText(
                                detect_frame, 
                                text, 
                                (text_x, text_y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5, 
                                (0, 0, 255),  # 빨간색
                                2
                            )   

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):  # Quit
                    self.destroy_node()
                    rclpy.shutdown()

                elif key == ord('a'):  # Detect AprilTag
                    self.get_logger().info("Key 'a' pressed: Detecting AprilTag")
                    _ = detect_apriltag(self.detector, detect_frame)

                elif key == ord('y'):  # Process both LiDAR and AprilTag coordinates
                    self.get_logger().info("Key 'y' pressed: Processing LiDAR and AprilTag coordinates")
                    cluster_frame = self.cluster_canvas.copy()
                    
                    apriltag_positions = detect_apriltag(self.detector, detect_frame)
                    result, camera_frame, cluster_frame = process_combined_coordinates(
                        apriltag_positions=apriltag_positions, 
                        camera_frame=detect_frame, 
                        cluster_frame=cluster_frame, 
                        apriltag_features=self.apriltag_features,
                        lidar_features=self.lidar_features,
                        cluster_tracker=self.cluster_tracker,
                        divide_num=self.config.divide_num, 
                        add_num=self.config.add_num)
                    
                    self.camera_frame = camera_frame
                    self.cluster_frame = cluster_frame

                    if not result:
                        self.get_logger().error("AprilTag의 좌표가 맞지 않습니다. 6개가 나와야함.")
                
                elif key == ord('b'):  # Load extrinsic calibration data
                    self.get_logger().info("Key 'b' pressed: Loading calibration_extrinsic.json if it exists.")
                    if os.path.exists(self.config.ex_calib_result_json):
                        self.rvec, self.tvec = load_extrinsic_from_json(self.config.ex_calib_result_json)
                        print("rvec: ", self.rvec)
                        print("tvec: ", self.tvec)

                        self.get_logger().info("Loaded rvec and tvec from calibration_extrinsic.json.")
                    else:
                        self.get_logger().error("calibration_extrinsic.json does not exist.")

                
                elif key == ord('c'):  # 개수 세기용
                    self.get_logger().info(f"Key 'c' pressed: lidar_features -> {len(self.lidar_features)}, apriltag_features -> {len(self.apriltag_features)}.")
                
                elif key == ord('e'):  # Calculate reprojection error
                    self.get_logger().info("Key 'e' pressed: Calculating reprojection error.")
                    self.calculate_and_save_reprojection_error()

                elif key == ord('z'):  # 캘리브레이션 진행
                    lidar_features = self.lidar_features
                    apriltag_features = self.apriltag_features
                    self.get_logger().info(f"Key 'z' pressed: Starting calibration with {len(lidar_features)} LiDAR features and {len(apriltag_features)} AprilTag features.")

                    # 캘리브레이션 외부행렬 추정
                    success, rvec, tvec = calibration_2dlidar_camera(
                        lidar_features, apriltag_features, self.camera_params.camera_matrix, self.camera_params.dist_coeffs
                    )

                    if success:
                        self.rvec = rvec
                        self.tvec = tvec
                        self.get_logger().info("Calibration successful. Saving extrinsic matrix.")
                        save_extrinsic_to_json(self.config.result_path , rvec, tvec)

                    
                    else:
                        self.get_logger().error("Calibration failed!")

            
                camera_frame_resized = cv2.resize(detect_frame, (self.cluster_canvas.shape[1], self.cluster_canvas.shape[0]))
                combined_canvas = np.hstack((camera_frame_resized, self.cluster_canvas))


                # Show combined canvas
                cv2.imshow("Camera and Clusters", combined_canvas)

            except Exception as e:
                self.get_logger().error(f"Failed to display combined image: {e}")

    def calculate_and_save_reprojection_error(self):
        """
        Calculate reprojection error and save to a CSV file.
        """
        if hasattr(self, 'rvec') and hasattr(self, 'tvec') and self.lidar_features and self.apriltag_features:
            try:
                # Calculate reprojection errors
                reprojection_errors = calculate_reprojection_error_2d(
                    self.camera_params.camera_matrix,
                    self.camera_params.dist_coeffs,
                    self.rvec,
                    self.tvec,
                    self.apriltag_features,
                    self.lidar_features
                )

                # Save errors to CSV
                csv_path = os.path.join(self.config.result_path, 'reprojection_errors.csv')
                save_reprojection_errors_to_csv(reprojection_errors, csv_path)

                self.get_logger().info(f"Reprojection errors calculated and saved to {csv_path}")

            except Exception as e:
                self.get_logger().error(f"Failed to calculate reprojection errors: {e}")


    def on_mouse_click(self, event, x, y, flags, param):
        """Handle mouse click events."""
        if event == cv2.EVENT_LBUTTONDOWN:
            # 왼쪽 카메라 창 너비 계산
            camera_width = self.cluster_canvas.shape[1]

            if x < camera_width:
                # 카메라 창에서 클릭
                self.get_logger().info(f"Mouse clicked in Camera window at ({x}, {y})")
            else:
                # LiDAR 창에서 클릭 (좌표 변환)
                lidar_x = x - camera_width
                self.get_logger().info(f"Mouse clicked in LiDAR window at ({lidar_x}, {y})")
                self.click_x = lidar_x
                self.click_y = y


    def destroy_node(self):
        """Cleanup on shutdown."""
        self.running = False
        self.capture.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()