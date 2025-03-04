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
from CalibJ.utils.config_loader import load_config, load_json
from CalibJ.utils.clustering_vis import points_to_pointcloud2
from CalibJ.module.clustering_scan import save_execution_statistics, dbscan_clustering, display_clusters, world_to_pixel # , optics_clustering
from CalibJ.module.calibration_module import calibration_2dlidar_camera
from CalibJ.module.apriltag_detect import detect_apriltag
from CalibJ.evaluate.clustering_eval import evaluate_clustering, record_evaluation_result 
from CalibJ.module.tracking import calculate_cluster_centers, ClusterTracker
import json
class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.config = load_config()
        self.get_logger().info(f"Loaded configuration: {self.config}")

        self.camera_params = load_json(self.config.cam_calib_result_json)

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
        labels, cluster_points, _ = dbscan_clustering(
            scan_data, epsilon=self.config.epsilon, min_samples=self.config.min_samples
        )

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
            canvas_size=800,
            cluster_tracker=self.cluster_tracker,
            only_clustering=False
        )

        self.camera_frame = camera_frame

    def show_visualization(self):
        """Main thread OpenCV GUI rendering."""
        if hasattr(self, 'cluster_canvas') and self.camera_frame is not None:
            try:
                detect_frame = self.camera_frame.copy()

                # LiDAR 데이터를 카메라 프레임에 투영
                if hasattr(self, 'extrinsic_matrix') and len(self.lidar_features) > 0:
                    lidar_points = np.array(self.pix_cluster_points)  # LiDAR 좌표 가져오기

                    # LiDAR x 좌표를 카메라 프레임 기준으로 이동
                    lidar_window_offset = self.cluster_canvas.shape[1]  # LiDAR 창 너비
                    lidar_points[:, 0] -= lidar_window_offset

                    print("Transformed lidar_points: ", lidar_points)  # 변환된 LiDAR 좌표 출력


                    projected_points = self.project_lidar_to_image(
                        lidar_points,
                        self.camera_params.camera_matrix,
                        self.camera_params.dist_coeffs,
                        self.extrinsic_matrix
                    )

                    # 투영된 점을 카메라 프레임 위에 그리기
                    for point in projected_points:
                        x, y = int(point[0]), int(point[1])
                        cv2.circle(detect_frame, (x, y), 3, (0, 255, 0), -1)  # 초록색 점으로 표시

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
                    result = self.process_combined_coordinates(detect_frame, cluster_frame, divide_num=self.config.divide_num, add_num=self.config.add_num)
                    if not result:
                        self.get_logger().error("AprilTag의 좌표가 맞지 않습니다. 6개가 나와야함.")

                elif key == ord('c'):  # 개수 세기용
                    self.get_logger().info(f"Key 'c' pressed: lidar_features -> {len(self.lidar_features)}, apriltag_features -> {len(self.apriltag_features)}.")

                elif key == ord('z'):  # 캘리브레이션 진행
                    lidar_features = self.lidar_features
                    apriltag_features = self.apriltag_features
                    self.get_logger().info(f"Key 'z' pressed: Starting calibration with {len(lidar_features)} LiDAR features and {len(apriltag_features)} AprilTag features.")

                    # 캘리브레이션 외부행렬 추정
                    success, rvec, tvec, extrinsic = calibration_2dlidar_camera(
                        lidar_features, apriltag_features, self.camera_params.camera_matrix, self.camera_params.dist_coeffs
                    )

                    if success:
                        self.extrinsic_matrix = extrinsic  # 외부행렬 저장
                        self.get_logger().info("Calibration successful. Saving extrinsic matrix.")
                        self.save_extrinsic_to_json(extrinsic)
                    else:
                        self.get_logger().error("Calibration failed!")

                camera_frame_resized = cv2.resize(detect_frame, (self.cluster_canvas.shape[1], self.cluster_canvas.shape[0]))
                combined_canvas = np.hstack((camera_frame_resized, self.cluster_canvas))

                # Show combined canvas
                cv2.imshow("Camera and Clusters", combined_canvas)

            except Exception as e:
                self.get_logger().error(f"Failed to display combined image: {e}")

    def project_lidar_to_image(self, lidar_points, camera_matrix, dist_coeffs, extrinsic_matrix):
        """
        LiDAR 데이터를 카메라 프레임으로 투영합니다.

        Args:
            lidar_points (np.ndarray): LiDAR 점들, shape=(N, 2).
            camera_matrix (np.ndarray): 카메라 내부행렬.
            dist_coeffs (np.ndarray): 카메라 왜곡 계수.
            extrinsic_matrix (np.ndarray): 외부행렬 (4x4).

        Returns:
            np.ndarray: 이미지 좌표, shape=(N, 2).
        """
        # 2D LiDAR 점들을 3D로 변환 (Z축 = 0)
        if lidar_points.shape[1] == 2:
            lidar_points = np.hstack((lidar_points, np.zeros((lidar_points.shape[0], 1))))  # Z = 0 추가

        # LiDAR 점들을 동차 좌표로 변환
        lidar_homogeneous = np.hstack((lidar_points, np.ones((lidar_points.shape[0], 1))))  # Homogeneous coordinates 추가

        # 외부행렬 적용
        camera_coords = lidar_homogeneous @ extrinsic_matrix.T  # (N, 4) x (4, 4)

        # 카메라 좌표계에서 이미지 평면으로 투영
        image_points = camera_coords[:, :3] / camera_coords[:, 2:3]  # Normalize by Z (N, 3)

        # 카메라 내부행렬 적용
        pixel_coords = (image_points @ camera_matrix.T)[:, :2]  # (N, 3) x (3, 3) -> (N, 2)

        # 왜곡 보정
        pixel_coords = cv2.undistortPoints(pixel_coords.reshape(-1, 1, 2), camera_matrix, dist_coeffs).reshape(-1, 2)

        return pixel_coords



    def save_extrinsic_to_json(self, extrinsic):
        """
        Save extrinsic matrix to a JSON file.

        Args:
            extrinsic (np.ndarray): The 4x4 extrinsic matrix.
        """
        result_path = os.path.join(self.config.result_path, "calibration_extrinsic.json")
        os.makedirs(self.config.result_path, exist_ok=True)

        calibration_data = {
            "extrinsic_matrix": extrinsic.tolist()
        }

        with open(result_path, 'w') as f:
            json.dump(calibration_data, f, indent=4)

        self.get_logger().info(f"Extrinsic matrix saved to {result_path}")

    def estimate_line(self, points):
        """
        주어진 점들로부터 직선을 추정합니다.
        
        Args:
            points (list or np.ndarray): N x 2 형태의 점들 (x, y).

        Returns:
            tuple: (기울기, y절편) 또는 None (점이 부족하거나 계산 실패 시).
        """
        if len(points) < 2:
            self.get_logger().error("Not enough points to estimate a line.")
            return None

        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]

        try:
            # NumPy의 polyfit을 사용하여 1차 다항식(직선) 적합
            m, c = np.polyfit(x, y, 1)  # m: 기울기, c: y절편
            return m, c
        except Exception as e:
            self.get_logger().error(f"Failed to estimate line with np.polyfit: {e}")
            return None



    def divide_line(self, line, divide_num):
        """
        직선을 일정 간격으로 나눕니다.

        Args:
            line (tuple): (기울기, y절편).
            divide_num (int): 나눌 구간 수.

        Returns:
            list: 분할된 점들의 리스트 [(x1, y1), (x2, y2), ...] 또는 None.
        """
        if line is None or divide_num < 2:
            self.get_logger().error("Invalid line or divide_num.")
            return None

        m, c = line
        x_start = 0
        x_end = 1  # 기본 범위를 0~1로 설정 (필요시 조정 가능)
        x_values = np.linspace(x_start, x_end, divide_num)
        points = [(x, m * x + c) for x in x_values]
        return points

    def process_combined_coordinates(self, camera_frame, cluster_frame, divide_num=15, add_num=3):
        """
        LiDAR와 AprilTag 좌표를 처리하여 각각 직선을 추정하고 일정 간격으로 나눕니다.

        Args:
            camera_frame (np.ndarray): 카메라 프레임.
            cluster_frame (np.ndarray): LiDAR 데이터를 시각화할 캔버스.
            divide_num (int): 직선을 나눌 구간 수.
            add_num (int): 추가 좌표의 개수.

        Returns:
            bool: 성공 여부
        """
        # 1. AprilTag 데이터 처리
        apriltag_positions = detect_apriltag(self.detector, camera_frame)

        if len(apriltag_positions) == 6:
            # 중앙 좌표 계산
            apriltag_centers = [
                (
                    tag[0],  # 태그 번호
                    ((tag[1][0] + tag[2][0]) // 2, (tag[1][1] + tag[2][1]) // 2)  # 중앙 좌표
                )
                for tag in apriltag_positions
            ]

            # 좌표 정렬 (중앙 좌표의 x 값을 기준으로 오름차순 정렬)
            apriltag_centers.sort(key=lambda x: x[1][0])

            # 정렬된 중앙 좌표 추출
            sorted_centers = [center[1] for center in apriltag_centers]

            # 중앙 좌표를 프레임에 그리기
            for center in sorted_centers:
                x, y = center
                cv2.circle(camera_frame, (x, y), 3, (0, 0, 255), 2)  # 안이 빈 빨간색 점

            # 맨 왼쪽과 맨 오른쪽 좌표 가져오기
            start_point = sorted_centers[0]
            end_point = sorted_centers[-1]
            x1, y1 = start_point
            x2, y2 = end_point

            # 직선 방정식 적합 (1차 함수 y = mx + c)
            A = np.vstack([np.array([x1, x2]), np.ones(2)]).T
            m, c = np.linalg.lstsq(A, np.array([y1, y2]), rcond=None)[0]

            # x 범위 계산 및 확장
            x_min = x1 - (x2 - x1) / (divide_num - 1) * add_num
            x_max = x2 + (x2 - x1) / (divide_num - 1) * add_num
            extended_x_range = np.linspace(x_min, x_max, divide_num + 2 * add_num)

            # 추가된 x 좌표로 y 좌표 계산
            extended_points = [(int(x), int(m * x + c)) for x in extended_x_range]

            # 선분 및 추가된 점을 프레임에 표시
            for point in extended_points:
                x, y = point
                cv2.circle(camera_frame, (x, y), 3, (255, 0, 0), 1)

            # 원래 선분을 노란색으로 시각화
            y_start = int(m * x1 + c)
            y_end = int(m * x2 + c)
            cv2.line(camera_frame, (int(x1), y_start), (int(x2), y_end), (0, 255, 255), 1)  

            # 추가된 점을 리스트에 저장
            self.apriltag_features.extend(extended_points)

            self.get_logger().info(f"AprilTag line divided into {len(extended_points)} points, including {add_num} extra points on each side.")

            # AprilTag에서 생성된 점의 개수를 저장
            total_apriltag_points = len(extended_points)
        else:
            self.get_logger().error("Invalid number of AprilTag positions detected.")
            return False

        # 2. LiDAR 데이터 처리
        if self.cluster_tracker.is_tracking:
            tracked_cluster_points = self.cluster_tracker.get_tracked_cluster_points()

            if tracked_cluster_points is None or len(tracked_cluster_points) < 2:
                self.get_logger().warn("Not enough points in the tracked cluster to create a line.")
                return False

            # LiDAR 좌표 정렬 (x 좌표 기준 오름차순)
            tracked_cluster_points = sorted(tracked_cluster_points, key=lambda p: p[0])

            # 맨 처음과 끝 좌표로 선분 생성
            start_point = tracked_cluster_points[0]
            end_point = tracked_cluster_points[-1]
            x1, y1 = start_point
            x2, y2 = end_point

            # 직선 방정식 적합 (1차 함수 y = mx + c)
            A = np.vstack([np.array([x1, x2]), np.ones(2)]).T
            m, c = np.linalg.lstsq(A, np.array([y1, y2]), rcond=None)[0]

            # LiDAR 점 생성 개수를 AprilTag와 동일하게 설정
            x_range = np.linspace(x1, x2, total_apriltag_points)

            # 나눠진 점 계산
            divided_points = [(int(x), int(m * x + c)) for x in x_range]

            # 카메라 프레임으로 좌표 변환
            lidar_window_offset = cluster_frame.shape[1]  # LiDAR 창 너비
            transformed_points = [(x - lidar_window_offset, y) for x, y in divided_points]

            # 나눠진 점을 클러스터 캔버스에 시각화
            for point in divided_points:
                x, y = point
                cv2.circle(cluster_frame, (x, y), 2, (0, 0, 255), 1)  # 빨간색 점으로 표시

            # 원래 선분을 노란색으로 시각화
            y_start = int(m * x1 + c)
            y_end = int(m * x2 + c)
            cv2.line(cluster_frame, (int(x1), y_start), (int(x2), y_end), (0, 255, 255), 1)  # 노란색 선

            # 변환된 점을 저장
            self.lidar_features.extend(transformed_points)

            self.get_logger().info(f"LiDAR line divided into {len(divided_points)} points and transformed to camera frame.")
        else:
            self.get_logger().warn("No cluster is currently being tracked.")
            return False

        # 업데이트된 프레임 저장
        self.camera_frame = camera_frame
        self.cluster_canvas = cluster_frame
        return True

        


    def on_mouse_click(self, event, x, y, flags, param):
        """Handle mouse click events."""
        if event == cv2.EVENT_LBUTTONDOWN:
            # 왼쪽 카메라 창 너비 계산
            camera_width = self.cluster_canvas.shape[1]

            if x < camera_width:
                # 카메라 창에서 클릭
                self.get_logger().info(f"Mouse clicked in Camera window at ({x}, {y})")
                self.click_x = x
                self.click_y = y
                # 필요한 경우 카메라 관련 추가 로직 작성 가능
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