import numpy as np
import cv2
import glob

# 체커보드 크기와 각 사각형의 크기
checkerboard_size = (8, 6)
square_size = 30  # mm

# 종료 기준 설정
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 체커보드의 3D 점 준비
objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
objp *= square_size  # 각 사각형의 크기를 곱하여 실제 크기 반영

# 객체 포인트와 이미지 포인트를 저장할 배열
objpoints = []  # 3D 포인트 (실제 세계)
imgpoints = []  # 2D 포인트 (이미지 평면)

# 체커보드 이미지 파일 불러오기
images = glob.glob('./*.jpg')

gray = None

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"Error: Could not load image {fname}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 체커보드 코너 찾기
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

    # 디버깅 메시지 추가
    if not ret:
        print(f"Could not find corners in {fname}")
        continue

    # 코너가 발견되면 포인트 저장
    objpoints.append(objp)
    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    imgpoints.append(corners2)

    # 코너 그리기 및 표시
    img = cv2.drawChessboardCorners(img, checkerboard_size, corners2, ret)
    cv2.imshow('img', img)
    cv2.waitKey(500)

cv2.destroyAllWindows()

# 카메라 캘리브레이션 수행
if gray is not None:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # 결과 출력
    if ret:
        print("Camera matrix:\n", mtx)
        print("Distortion coefficients:\n", dist)

        # 예제: 이미지에서 왜곡 제거
        for fname in images:
            img = cv2.imread(fname)
            h, w = img.shape[:2]
            new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

            # 왜곡 제거
            dst = cv2.undistort(img, mtx, dist, None, new_camera_mtx)

            # 결과 표시
            cv2.imshow('Undistorted Image', dst)
            cv2.waitKey(500)

    cv2.destroyAllWindows()
else:
    print("Error: No valid gray image found for calibration.")
