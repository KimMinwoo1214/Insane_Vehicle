import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
import pandas as pd
import numpy as np


class FastAngleNode(Node):
    def __init__(self):
        super().__init__('gps_angle_publisher_fast')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.publisher_ = self.create_publisher(String, '/gps_steering_angle', 10)

        self.xy = self.create_subscription(
            String,
            'gps',
            self.xy_callback,
            qos
        )

        self.declare_parameter('tolerance', 1.5)
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value

        # 이전 좌표값을 저장하기 위한 변수 초기화
        self.prev_x = None
        self.prev_y = None

        file_path = '/home/moon/Desktop/ublox_dgnss/output.csv'
        try:
            df = pd.read_csv(file_path)
            self.points = df[["UTM_X(East)", "UTM_Y(North)"]].to_numpy()
            self.get_logger().info(f"✅ Loaded {self.points.shape[0]} points.")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load CSV: {e}")
            self.points = np.empty((0, 2))

    def xy_callback(self, msg):
        # 메시지에서 X, Y 좌표만 파싱합니다.
        data_parts = msg.data.split(',')
        if len(data_parts) < 2:
            self.get_logger().warn(f"Invalid GPS message (expected X,Y,...): {msg.data}")
            return

        try:
            x = float(data_parts[0])
            y = float(data_parts[1])
        except ValueError:
            self.get_logger().warn(f"Could not parse coordinates: {msg.data}")
            return

        # 1. 이전 좌표가 없는 경우 (첫 메시지 수신 시)
        # 현재 좌표를 이전 좌표로 저장하고 콜백을 종료합니다.
        if self.prev_x is None or self.prev_y is None:
            self.prev_x = x
            self.prev_y = y
            self.get_logger().info("📍 Initial coordinates set. Waiting for next message to calculate direction.")
            self.publisher_.publish(String(data='-1'))
            return

        # 2. 이전 좌표와 현재 좌표를 이용해 이동 방향 벡터(dx, dy)를 계산합니다.
        dx = x - self.prev_x
        dy = y - self.prev_y

        # 3. 다음 계산을 위해 현재 좌표를 이전 좌표로 업데이트합니다.
        self.prev_x = x
        self.prev_y = y

        # 이동 거리가 매우 작으면 방향 계산이 무의미하므로 건너뜁니다.
        if np.sqrt(dx**2 + dy**2) < 0.01: # 1cm 미만 이동은 무시
            self.get_logger().info("Movement too small to determine direction.")
            return

        # 4. 경로 탐색 로직
        # 주변 경로 포인트 필터링
        diff = self.points - np.array([x, y])
        mask = (np.abs(diff[:, 0]) <= self.tolerance) & (np.abs(diff[:, 1]) <= self.tolerance)
        filtered_points = self.points[mask]
        filtered_diff = diff[mask]

        if filtered_points.shape[0] == 0:
            self.get_logger().warn("No points within tolerance.")
            self.publisher_.publish(String(data='-1'))
            return

        # 계산된 이동 방향 벡터(dx, dy)를 사용합니다.
        dir_vec = np.array([dx, dy])
        t_values = np.dot(filtered_diff, dir_vec)

        # 이동 방향의 뒤쪽에 있는 포인트는 제외합니다 (t <= 0).
        t_values[t_values <= 0] = np.inf

        min_t_idx = np.argmin(t_values)
        if np.isinf(t_values[min_t_idx]):
            self.get_logger().warn("No valid point in the direction of travel.")
            self.publisher_.publish(String(data='-1'))
            return

        # 5. 목표 지점 및 기울기 계산
        closest_point = filtered_points[min_t_idx]

        delta_x = closest_point[0] - x
        delta_y = closest_point[1] - y

        if delta_x == 0:
            slope = float('inf')
        else:
            slope = delta_y / delta_x

        self.get_logger().info(f"Slope (coordinate-based): {slope:.6f}")
        self.publisher_.publish(String(data=str(slope)))


def main():
    rclpy.init()
    node = FastAngleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()