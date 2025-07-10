import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class LanePlanner(Node):
    def __init__(self):
        super().__init__('lane_planner_node')
        self.img_width = 640
        self.img_height = 480
        self.latest_img = None
        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self.sub_polygon = self.create_subscription(
            PolygonStamped, '/yolo_polygon', self.polygon_callback, qos)
        self.sub_img = self.create_subscription(
            Image, '/video_frames', self.image_callback, qos)
        self.pub_angle = self.create_publisher(Float32, '/lane_steering_angle', qos)

    def image_callback(self, msg):
        try:
            self.latest_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")

    def polygon_callback(self, msg):
        if self.latest_img is None:
            self.get_logger().warn("⚠️ 이미지 수신 전, 스킵")
            return
        if len(msg.polygon.points) < 4:
            self.get_logger().warn("⚠️ 폴리곤 점 부족, 스킵")
            return

        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.int32)
        overlay = self.latest_img.copy()

        # 1) 왼쪽 차선선
        cv2.line(overlay, tuple(pts[0]), tuple(pts[1]), (0,255,255), 2)
        # 2) 오른쪽 차선선
        cv2.line(overlay, tuple(pts[3]), tuple(pts[2]), (255,0,255), 2)
        # 3) 중앙 기준선
        mid_x = self.img_width // 2
        cv2.line(overlay, (mid_x, self.img_height), (mid_x, 0), (128,128,128), 1)

        # 마지막 교차점 및 해당 세그먼트 계산
        inter, seg = self.find_last_intersection_and_tangent(pts, ((mid_x, self.img_height),(mid_x,0)))
        if inter and seg:
            # 조향 각도 계산 및 퍼블리시
            angle = self.calculate_tangent_angle(seg)
            msg = Float32()
            msg.data = float(angle)
            self.pub_angle.publish(msg)
            self.get_logger().info(f"✅ Angle: {angle:.2f}")

            # 얇은 빨간색 각도선
            start = (mid_x, self.img_height)
            length = 50
            rad = np.deg2rad(angle + 90)
            end = (int(start[0] + length*np.cos(rad)), int(start[1] - length*np.sin(rad)))
            cv2.line(overlay, start, end, (0,0,255), 1)

        # 화면에 출력
        cv2.imshow("Lane Planner", overlay)
        cv2.waitKey(1)

    def find_last_intersection_and_tangent(self, pts, line):
        inters = []
        for i in range(len(pts)):
            p1, p2 = pts[i], pts[(i+1)%len(pts)]
            ip = self.line_intersection((p1,p2), line)
            if ip:
                inters.append({'pt':ip,'seg':(p1,p2)})
        if not inters:
            return None, None
        last = max(inters, key=lambda x: x['pt'][1])
        return last['pt'], last['seg']

    def line_intersection(self, l1, l2):
        (x1,y1),(x2,y2) = l1
        (x3,y3),(x4,y4) = l2
        den = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)
        if abs(den)<1e-6: return None
        t = ((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/den
        u = -((x1-x2)*(y1-y3)-(y1-y2)*(x1-x3))/den
        if 0<=t<=1 and 0<=u<=1:
            return (x1+t*(x2-x1), y1+t*(y2-y1))
        return None

    def calculate_tangent_angle(self, seg):
        (x1,y1),(x2,y2) = seg
        rad = np.arctan2(x2-x1, -(y2-y1))
        return np.rad2deg(rad)

def main(args=None):
    rclpy.init(args=args)
    node = LanePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__=='__main__':
    main()
