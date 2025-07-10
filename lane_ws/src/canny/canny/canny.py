#!/usr/bin/env python3
import cv2
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32
from cv_bridge import CvBridge
from concurrent.futures import ThreadPoolExecutor

class Canny(Node):
    def __init__(self):
        super().__init__('canny_node')
        self.bridge = CvBridge()
        self.display_img = None
        self.roi_visual = None
        self.thread_pool = ThreadPoolExecutor(max_workers=1)

        cv2.namedWindow('ROI Mask', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Canny Result', cv2.WINDOW_NORMAL)

        # 100Hz 타이머
        self.create_timer(0.01, self.process_frame)

        # QoS 설정
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self.sub_img = self.create_subscription(Image, '/video_frames', self.image_callback, qos)
        self.poly_pub = self.create_publisher(PolygonStamped, '/canny_polygons', qos)

    @staticmethod
    def convex_hull(pts: np.ndarray) -> np.ndarray:
        pts = pts[np.lexsort((pts[:,1], pts[:,0]))]
        if len(pts) <= 2:
            return pts
        def build_half(arr):
            hull = []
            for p in arr:
                while len(hull) >= 2:
                    a, b = hull[-2], hull[-1]
                    cross = (b[0]-a[0])*(p[1]-a[1]) - (b[1]-a[1])*(p[0]-a[0])
                    if cross <= 0:
                        hull.pop()
                    else:
                        break
                hull.append(tuple(p))
            return hull
        lower = build_half(pts)
        upper = build_half(pts[::-1])
        hull = lower[:-1] + upper[:-1]
        return np.array(hull, dtype=np.int32)

    def image_callback(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0,0,180), (180,30,255))
            masked = cv2.bitwise_and(img, img, mask=mask)

            h, w = masked.shape[:2]
            trapezoid = np.array([[
                (int(0.1*w), h),
                (int(0.9*w), h),
                (int(0.6*w), int(0.6*h)),
                (int(0.4*w), int(0.6*h))
            ]], dtype=np.int32)

            roi_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.fillPoly(roi_mask, trapezoid, 255)
            roi = cv2.bitwise_and(masked, masked, mask=roi_mask)

            vis = masked.copy()
            cv2.polylines(vis, trapezoid, True, (0,0,255), 2)
            self.roi_visual = vis

            # CPU Canny 처리 스레드에 전달
            self.thread_pool.submit(self.detect_lane, roi.copy(), time.time())
        except Exception as e:
            self.get_logger().error(f'[image_callback] {e}')

    def detect_lane(self, image: np.ndarray, t_start: float):
        try:
            # OpenCV CPU Canny Edge
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            pts = np.column_stack(np.nonzero(edges))[:, ::-1]
            if pts.size == 0:
                return

            hull = self.convex_hull(pts)
            vis = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            cv2.polylines(vis, [hull.reshape(-1,1,2)], True, (0,255,0), 2)

            # 처리 시간 표시
            t_ms = (time.time() - t_start) * 1000.0
            cv2.putText(vis,
                        f'{t_ms:.1f} ms',
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0,255,255),
                        2)

            # PolygonStamped 퍼블리시
            poly_msg = PolygonStamped()
            poly_msg.header.stamp = self.get_clock().now().to_msg()
            poly_msg.header.frame_id = 'camera_frame'
            poly_msg.polygon.points = [
                Point32(x=float(x), y=float(y), z=0.0)
                for x, y in hull
            ]
            self.poly_pub.publish(poly_msg)

            self.display_img = vis

        except Exception as e:
            self.get_logger().error(f'[detect_lane] {e}')

    def process_frame(self):
        if self.roi_visual is not None:
            cv2.imshow('ROI Mask', self.roi_visual)
        if self.display_img is not None:
            cv2.imshow('Canny Result', self.display_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Canny()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
