import numpy as np
import csv 
import cv2

def calculate_reprojection_error_2d(camera_matrix, dist_coeffs, rvec, tvec, original_points, lidar_points):
    """
    Calculate reprojection error for 2D LiDAR calibration.

    Args:
        camera_matrix (np.ndarray): Camera intrinsic matrix.
        dist_coeffs (np.ndarray): Camera distortion coefficients.
        rvec (np.ndarray): Rotation vector (camera to LiDAR).
        tvec (np.ndarray): Translation vector (camera to LiDAR).
        original_points (list): Original 2D image points.
        lidar_points (list): Corresponding 2D LiDAR points (assumed Z=0).

    Returns:
        errors (list): List of reprojection errors for each point.
    """
    # 2D LiDAR 좌표에 Z=0 추가
    lidar_points_3d = np.hstack([np.array(lidar_points), np.zeros((len(lidar_points), 1))])

    # LiDAR 3D 포인트를 2D 이미지 좌표로 투영
    projected_points, _ = cv2.projectPoints(
        lidar_points_3d, rvec, tvec, camera_matrix, dist_coeffs
    )
    projected_points = projected_points.squeeze()

    # 각 포인트의 유클리드 거리 계산
    errors = np.linalg.norm(np.array(original_points) - np.array(projected_points), axis=1)
    return errors

def save_reprojection_errors_to_csv(errors, filepath):
    """
    Save reprojection errors to a CSV file.

    Args:
        errors (list): List of reprojection errors.
        filepath (str): Path to the CSV file.
    """
    header = ['Point Index', 'Reprojection Error']
    with open(filepath, mode='a', newline='') as file:
        writer = csv.writer(file)
        if file.tell() == 0:  # Write header only if file is empty
            writer.writerow(header)
        for idx, error in enumerate(errors):
            writer.writerow([idx, error])
