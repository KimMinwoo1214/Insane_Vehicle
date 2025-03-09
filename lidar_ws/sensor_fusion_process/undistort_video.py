import cv2
import numpy as np

# 🎥 비디오 캡처 (2번 카메라 사용)
cap = cv2.VideoCapture(2)

# 카메라 행렬 (Camera Matrix)
K = np.array([[703.37906585, 0., 330.37487405],
              [0., 750.72854219, 226.5012125],
              [0., 0., 1.]])

# 왜곡 계수 (Distortion Coefficients)
dist_coeffs = np.array([8.65114817e-02, 5.75780539e-01, -3.92050613e-03, 2.34661487e-03, -2.74255703e+00])

# 비디오 스트림 실행
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("❌ 비디오 프레임을 가져올 수 없습니다!")
        break

    # 프레임 크기
    h, w = frame.shape[:2]

    # 보정된 카메라 행렬 계산 (첫 실행 시 1회만 계산)
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(K, dist_coeffs, (w, h), 1, (w, h))

    # 실시간 프레임 왜곡 제거
    undistorted_frame = cv2.undistort(frame, K, dist_coeffs, None, new_camera_matrix)

    # ROI 적용하여 크롭 (불필요한 검은 부분 제거)
    x, y, w, h = roi
    undistorted_frame = undistorted_frame[y:y+h, x:x+w]

    # 📺 실시간 영상 출력
    cv2.imshow("Undistorted Video", undistorted_frame)

    # ESC 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == 27:
        break

# 정리
cap.release()
cv2.destroyAllWindows()
