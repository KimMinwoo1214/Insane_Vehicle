import cv2
import numpy as np

# ✅ 🎯 2번 카메라 연결
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("❌ 카메라를 열 수 없습니다!")
    exit()

# ✅ 🎯 카메라 내부 행렬 (Camera Matrix)
camera_intrinsics = np.array([
    [703.37906585, 0., 330.37487405],
    [0., 750.72854219, 226.5012125],
    [0., 0., 1.]
])

# ✅ 🎯 LiDAR → 카메라 변환 행렬 (T_lc)
T_lc = np.array([
    [-0.83120904,  0.49408048,  0.25490393, -0.27312576],
    [ 0.17381458, -0.2045658,   0.96329711,  0.04575654],
    [ 0.52809093,  0.84500729,  0.08415855,  0.30641326],
    [ 0, 0, 0, 1]
])

# ✅ 🎯 LiDAR 좌표 (X_L, Y_L, Z_L)
lidar_coords = np.array([
    [-0.41491, -0.0088547, 0],
    [-0.4109,  0.04159, 0],
    [-0.37611, -0.08226, 0],
    [-0.41282, -0.13064, 0],
    [-0.38516, -0.17966, 0],
    [-0.38516, -0.17966, 0]
], dtype=np.float32)

# ✅ LiDAR 데이터 동차 좌표 변환 (Homogeneous Coordinates)
ones = np.ones((lidar_coords.shape[0], 1))
lidar_homogeneous = np.hstack((lidar_coords, ones))

while True:
    # 🎥 실시간 카메라 프레임 읽기
    ret, frame = cap.read()
    if not ret:
        print("❌ 프레임을 가져올 수 없습니다!")
        break

    h, w = frame.shape[:2]  # 프레임 크기 확인

    # ✅ LiDAR → 카메라 좌표 변환
    lidar_camera = (T_lc @ lidar_homogeneous.T).T  # 4x4 변환 적용

    # ✅ 정규화 (카메라 좌표 → 이미지 픽셀 좌표 변환)
    projected_points = camera_intrinsics @ lidar_camera[:, :3].T  # 3x3 곱셈
    projected_points /= projected_points[2]  # 정규화

    # ✅ 최종 픽셀 좌표
    image_points = projected_points[:2].T.astype(int)

    # ✅ LiDAR 포인트를 프레임에 오버레이
    for (u, v) in image_points:
        if 0 <= u < w and 0 <= v < h:  # 이미지 범위를 벗어나지 않도록 확인
            cv2.circle(frame, (u, v), 5, (0, 0, 255), -1)  # 🔴 빨간 점 표시

    # 📺 결과 출력
    cv2.imshow("LiDAR Points on Camera", frame)

    # ESC 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == 27:
        break

# ✅ 정리
cap.release()
cv2.destroyAllWindows()
