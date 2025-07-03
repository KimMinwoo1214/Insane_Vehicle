from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class YOLOSegNode(Node):
    def __init__(self):
        super().__init__('yolo_seg_node')
        self.bridge = CvBridge()
        self.model = YOLO("train1/weights/best.pt")
        self.subscription = self.create_subscription(Image, '/video_frames', self.callback, 10)
        self.get_logger().info("✅ YOLO 세그멘테이션 노드 시작됨.")

    def callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(img)[0]
            result_img = results.plot()
            cv2.imshow("YOLO Segmentation", result_img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"❌ 추론 중 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
