import cv2

def capture_and_save():
    cap = cv2.VideoCapture(2)  # 기본 카메라(웹캠) 실행
    
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break
        
        cv2.imshow("Camera", frame)  # 영상 출력
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):  # 's' 키를 누르면 사진 저장
            cv2.imwrite("captured_image.jpg", frame)
            print("Image saved as captured_image.jpg")
        elif key == ord('q'):  # 'q' 키를 누르면 종료
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_and_save()
