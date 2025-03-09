import cv2
import numpy as np
from scipy.spatial.transform import Rotation

# 🎯 주어진 카메라 내부 행렬 (Camera Matrix)
camera_intrinsics = np.array([
    [703.37906585, 0., 330.37487405],
    [0., 750.72854219, 226.5012125],
    [0., 0., 1.]
])

# 🎯 왜곡 계수 (Distortion Coefficients)
dist_coeffs = np.array([[8.65114817e-02, 5.75780539e-01, -3.92050613e-03, 2.34661487e-03, -2.74255703e+00]])

# 🎯 LiDAR 점 (X, Y, Z)
lidar_coords = np.array([
    [-0.41491, -0.0088547, 0],
    [-0.4109, 0.04159, 0],
    [-0.37611, -0.08226, 0],
    [-0.41282, -0.13064, 0],
    [-0.38516, -0.17966, 0],
    [-0.38516, -0.17966, 0]
], dtype=np.float32)

# 🎯 카메라 픽셀 좌표 (u, v)
image_coords = np.array([
    [399, 210],
    [317, 210],
    [213, 210],
    [122, 210],
    [27, 210],
    [485, 210]
], dtype=np.float32)

# 🎯 이미지 좌표 왜곡 보정
undistorted_points = cv2.undistortPoints(image_coords, camera_intrinsics, dist_coeffs)

# 🎯 왜곡 보정된 픽셀 좌표를 2D에서 3D로 변환
image_points_3d = np.zeros((undistorted_points.shape[0], 3), dtype=np.float32)
image_points_3d[:, 0] = undistorted_points[:, 0, 0]
image_points_3d[:, 1] = undistorted_points[:, 0, 1]
image_points_3d[:, 2] = 1.0  # 깊이값(기본 설정)

# 🎯 SolvePnP로 변환 행렬 계산
retval, rvec, tvec = cv2.solvePnP(lidar_coords, image_coords, camera_intrinsics, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

# 🎯 회전 벡터를 회전 행렬로 변환
rotation_matrix, _ = cv2.Rodrigues(rvec)

# 🎯 변환 행렬 출력
print("\n✅ Extrinsics (LiDAR → Camera) ✅")
print("Translation Vector (T) = \n", tvec)
print("\nRotation Matrix (R) = \n", rotation_matrix)

# 🎯 오일러 각 변환 (Yaw-Pitch-Roll)
rot_mat = Rotation.from_matrix(rotation_matrix)
xyz_angles = rot_mat.as_euler('xyz', degrees=True)

print("\n🔄 Euler Angles (XYZ):")
print(f"Roll: {xyz_angles[0]:.2f}°")
print(f"Pitch: {xyz_angles[1]:.2f}°")
print(f"Yaw: {xyz_angles[2]:.2f}°")

# 🎯 4x4 변환 행렬 생성
T_lc = np.eye(4)
T_lc[:3, :3] = rotation_matrix
T_lc[:3, 3] = tvec.flatten()

print("\n🔄 Homogeneous Transformation Matrix (T_lc):")
print(T_lc)
