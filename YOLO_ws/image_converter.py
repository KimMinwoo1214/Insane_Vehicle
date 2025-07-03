import cv2
import numpy as np
import os

if __name__ == "__main__":
    src_dir  = "/Users/parkm04/PycharmProjects/Lane/Curvelanes/train"
    os.makedirs(f"{src_dir}/processed", exist_ok=True)

    # HSV에서 흰색 범위
    white_lower = np.array([0, 0, 200], dtype=np.uint8)
    white_upper = np.array([180, 30, 255], dtype=np.uint8)

    for fname in os.listdir(src_dir):
        src_path = os.path.join(src_dir, fname)
        img = cv2.imread(src_path)
        if img is None:
            continue

        # HSV 변환 후 마스크 생성
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, white_lower, white_upper)
        roi  = cv2.bitwise_and(img, img, mask=mask)

        # 결과 저장
        out_path = os.path.join(f"{src_dir}/processed", fname)
        cv2.imwrite(out_path, roi, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
