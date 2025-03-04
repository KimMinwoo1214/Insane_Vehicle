import numpy as np
import cv2
from CalibJ.module.apriltag_detect import detect_apriltag

def calibration_2dlidar_camera(lidar_features, apriltag_features, camera_matrix, dist_coeffs):
    """
    Perform calibration between 2D LiDAR and camera using corresponding points.

    Args:
        lidar_features (list of tuples): LiDAR feature points in world coordinates (2D).
        apriltag_features (list of tuples): AprilTag feature points in image coordinates.
        camera_matrix (np.ndarray): Camera intrinsic matrix (3x3).
        dist_coeffs (np.ndarray): Camera distortion coefficients (1D array).

    Returns:
        tuple:
            success (bool): Whether the calibration succeeded.
            rvec (np.ndarray): Rotation vector (3x1).
            tvec (np.ndarray): Translation vector (3x1).
            extrinsic (np.ndarray): Extrinsic matrix (4x4).
    """
    # Ensure the inputs are numpy arrays
    lidar_features = np.array(lidar_features, dtype=np.float32)
    apriltag_features = np.array(apriltag_features, dtype=np.float32)

    # Convert 2D LiDAR points to 3D by setting Z = 0
    lidar_features_3d = np.hstack((lidar_features, np.zeros((lidar_features.shape[0], 1))))  # Add Z = 0

    # Check if the number of points matches
    if lidar_features_3d.shape[0] != apriltag_features.shape[0]:
        raise ValueError("The number of LiDAR features and AprilTag features must be the same.")

    # SolvePnP to estimate the rotation and translation vectors
    success, rvec, tvec = cv2.solvePnP(
        lidar_features_3d,  # 3D world points (LiDAR)
        apriltag_features,  # 2D image points (AprilTag)
        camera_matrix,  # Camera intrinsic matrix
        dist_coeffs,  # Camera distortion coefficients
        flags=cv2.SOLVEPNP_ITERATIVE
    )

    if not success:
        return False, None, None

    return success, rvec, tvec

def process_combined_coordinates(apriltag_positions, camera_frame, cluster_frame, apriltag_features, lidar_features, cluster_tracker, divide_num=15, add_num=3):
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
            apriltag_features.extend(extended_points)

            print(f"AprilTag line divided into {len(extended_points)} points, including {add_num} extra points on each side.")

            # AprilTag에서 생성된 점의 개수를 저장
            total_apriltag_points = len(extended_points)
        else:
            print("Invalid number of AprilTag positions detected.")
            return False, camera_frame, cluster_frame,

        # 2. LiDAR 데이터 처리
        if cluster_tracker.is_tracking:
            tracked_cluster_points = cluster_tracker.get_tracked_cluster_points()

            if tracked_cluster_points is None or len(tracked_cluster_points) < 2:
                print("Not enough points in the tracked cluster to create a line.")
                return False, camera_frame, cluster_frame,

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
   
            # 나눠진 점을 클러스터 캔버스에 시각화
            for point in divided_points:
                x, y = point
                cv2.circle(cluster_frame, (x, y), 2, (0, 0, 255), 1)  # 빨간색 점으로 표시

            # 원래 선분을 노란색으로 시각화
            y_start = int(m * x1 + c)
            y_end = int(m * x2 + c)
            cv2.line(cluster_frame, (int(x1), y_start), (int(x2), y_end), (0, 255, 255), 1)  # 노란색 선

            # 변환된 점을 저장
            lidar_features.extend(divided_points)

            print(f"LiDAR line divided into {len(divided_points)} points and transformed to camera frame.")
        else:
            print("No cluster is currently being tracked.")
            return False, camera_frame, cluster_frame,

        # 업데이트된 프레임 저장
        return True, camera_frame, cluster_frame


def project_lidar_to_image(lidar_points, camera_matrix, dist_coeffs, rvec, tvec):
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

        # 외부행렬 분리 (회전 및 변환)
        rotation_matrix = rvec
        translation_vector = tvec

        # cv2.projectPoints 사용
        lidar_points = lidar_points.reshape(-1, 1, 3)  # (N, 3) -> (N, 1, 3)
        pixel_coords, _ = cv2.projectPoints(
            lidar_points,
            rotation_matrix,
            translation_vector,
            camera_matrix,
            dist_coeffs
        )

        return pixel_coords.reshape(-1, 2)  # (N, 1, 2) -> (N, 2)


def convert_image_to_lidar(image_width, image_height, camera_matrix, dist_coeffs, rvec, tvec):
    """
    Convert image points to LiDAR frame coordinates.
    
    Args:
        image_corners (np.ndarray): 2D array of image points (Nx2).
        camera_matrix (np.ndarray): Intrinsic camera matrix.
        dist_coeffs (np.ndarray): Distortion coefficients of the camera.
        rvec (np.ndarray): Rotation vector from camera to LiDAR.
        tvec (np.ndarray): Translation vector from camera to LiDAR.

    Returns:
        lidar_points (np.ndarray): 3D array of points in the LiDAR frame (Nx3).
    """

    image_corners = np.array([
        [0, 0],
        [image_width - 1, 0],
        [image_width - 1, image_height - 1],
        [0, image_height - 1],
    ], dtype=np.float32)


    # Undistort image points
    undistorted_points = cv2.undistortPoints(
        np.expand_dims(image_corners, axis=1), camera_matrix, dist_coeffs
    )

    # Convert rotation vector to rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rvec)

    # Convert to homogeneous coordinates
    undistorted_points_homo = np.hstack((undistorted_points.squeeze(), np.zeros((undistorted_points.shape[0], 1))))

    # Back-project points into camera 3D coordinates (assuming Z=0 for scale)
    camera_points = np.dot(np.linalg.inv(camera_matrix), undistorted_points_homo.T).T

    # Transform camera points to LiDAR frame
    lidar_points = np.dot(rotation_matrix.T, (camera_points - tvec.T).T).T

    return lidar_points