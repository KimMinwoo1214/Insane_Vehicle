import cv2
import matplotlib.pyplot as plt

clicked_points = []

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_points.append((x, y))
        print(f"Point {len(clicked_points)}: ({x}, {y})")

        # 표시
        img_copy = param.copy()
        for pt in clicked_points:
            cv2.circle(img_copy, pt, 5, (0, 255, 0), -1)
        cv2.imshow("Click 4 points (clockwise)", img_copy)

        # 완료
        if len(clicked_points) == 4:
            print("\n==== Final src_mat (BEV 원본) ====")
            print("[")
            for pt in clicked_points:
                print(f"  [{pt[0]}, {pt[1]}],")
            print("]")
            cv2.destroyAllWindows()

def main():
    img_path = "your_image.jpg"  # << 이미지 경로 수정 필요
    img = cv2.imread(img_path)
    if img is None:
        print("이미지 로드 실패. 경로를 확인하세요.")
        return

    cv2.imshow("Click 4 points (clockwise)", img)
    cv2.setMouseCallback("Click 4 points (clockwise)", mouse_callback, img)

    print("이미지에서 좌상단 → 우상단 → 우하단 → 좌하단 순으로 클릭하세요.")
    cv2.waitKey(0)

if __name__ == "__main__":
    main()
