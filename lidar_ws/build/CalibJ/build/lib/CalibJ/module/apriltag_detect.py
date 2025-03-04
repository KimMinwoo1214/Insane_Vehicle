from aprilgrid import Detector
import cv2

def detect_apriltag(detector, frame):
    april_2d_positions = []

    # 이미지 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray)

    # 검출된 태그 순회
    for detection in detections:
        tag_id = detection.tag_id
        corners = detection.corners

        corner_01 = (int(corners[0][0][0]), int(corners[0][0][1]))
        corner_02 = (int(corners[1][0][0]), int(corners[1][0][1]))
        corner_03 = (int(corners[2][0][0]), int(corners[2][0][1]))
        corner_04 = (int(corners[3][0][0]), int(corners[3][0][1]))

        center_x = int((corner_01[0] + corner_02[0] + corner_03[0] + corner_04[0]) / 4)
        center_y = int((corner_01[1] + corner_02[1] + corner_03[1] + corner_04[1]) / 4)
        tag_center = (center_x, center_y)

        cv2.line(frame, corner_01, corner_02, (255, 0, 0), 1)
        cv2.line(frame, corner_02, corner_03, (255, 102, 0), 1)
        cv2.line(frame, corner_03, corner_04, (0, 255, 102), 1)
        cv2.line(frame, corner_04, corner_01, (0, 255, 0), 1)

        cv2.putText(frame, str(tag_id), (center_x - 10, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.circle(frame, tag_center, 3, (0, 0, 255), 1)

        if tag_id in [6, 7, 8, 9, 10, 11]:
            april_2d_positions.append((tag_id, corner_01, corner_02))   # 태그의 좌, 우 하단의 모서리 좌표 추가

    return april_2d_positions
    