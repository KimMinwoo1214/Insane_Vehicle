import cv2
import os
from datetime import datetime


def save_webcam_frames():
    # Create testdata directory if it doesn't exist
    if not os.path.exists('testdata'):
        os.makedirs('testdata')

    # Initialize webcam
    cap = cv2.VideoCapture(2)

    try:
        if not cap.isOpened():
            raise IOError("Cannot open webcam")

        print("Press 'q' to quit...")
        while True:
            # Read a frame
            ret, frame = cap.read()

            if ret:
                # Generate filename with timestamp
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f'testdata/frame_{timestamp}.jpg'

                # Save the frame
                cv2.imwrite(filename, frame)
                print(f"Frame saved as {filename}")

                # Display the frame
                cv2.imshow('Webcam', frame)

                # Break loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Failed to capture frame")
                break

    except KeyboardInterrupt:
        print("\nStopping capture...")
    finally:
        # Release resources
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    save_webcam_frames()
