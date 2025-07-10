import cv2
import numpy as np
import glob

# 체스보드 크기 (예: 9x6)
chessboard_size = (9, 6)
square_size = 25  # 체스보드 정사각형 한 변의 길이 (mm 단위)

# 3D 좌표 준비 (예: (0,0,0), (1,0,0), (2,0,0), ..., (8,5,0))
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size  # 정사각형 크기 반영

# 이미지 저장할 리스트
obj_points = []  # 3D 좌표
img_points = []  # 2D 좌표

# 캘리브레이션할 이미지 불러오기
images = glob.glob("*.jpg")  # 폴더 내 이미지 경로

gray = None  # 초기화 (최소한 하나의 이미지에서 gray를 정의해야 함)

if len(images) == 0:
    print("Error: No calibration images found!")
else:
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # `gray` 정의

        # 체스보드 코너 찾기
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if ret:
            obj_points.append(objp)
            img_points.append(corners)

            # 코너 그리기
            img = cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
            cv2.imshow("Calibration Image", img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()

    # 최소한 하나의 이미지가 유효해야 캘리브레이션 수행 가능
    if gray is not None and len(obj_points) > 0:
        # 카메라 캘리브레이션 실행
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            obj_points, img_points, gray.shape[::-1], None, None
        )

        # 결과 출력
        print("Camera Matrix:\n", camera_matrix)
        print("Distortion Coefficients:\n", dist_coeffs)

        # 보정된 이미지 확인
        img = cv2.imread(images[0])  # 첫 번째 이미지 테스트
        h, w = img.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))

        # 이미지 보정
        dst = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)

        # 보정된 이미지 저장
        cv2.imwrite("calibrated_image.jpg", dst)
        cv2.imshow("Undistorted Image", dst)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # 보정된 카메라 매트릭스 저장
        np.savez("camera_calibration_data.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, new_camera_matrix=new_camera_matrix)
    else:
        print("Error: No valid chessboard images found for calibration!")
