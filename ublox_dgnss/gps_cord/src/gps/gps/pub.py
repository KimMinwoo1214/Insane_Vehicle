#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LatLonFilePublisher(Node):
    def __init__(self):
        super().__init__('lat_lon_file_publisher')
        # 파일 경로를 파라미터로 선언 및 취득
        self.declare_parameter('file_path', '/home/moon/Desktop/ublox_dgnss/realest.txt')
        file_path = self.get_parameter('file_path').value

        # 데이터 리스트 읽기 및 변환
        self.data_list = self.load_and_swap(file_path)
        if not self.data_list:
            self.get_logger().error(f"No valid data loaded from '{file_path}'")
            rclpy.shutdown()
            return

        # 퍼블리셔 생성
        self.publisher = self.create_publisher(String, 'gps_bak', 10)

        # 인덱스 초기화
        self._index = 0

        # 0.03초(30ms)마다 콜백 실행
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.get_logger().info(f"Publishing {len(self.data_list)} entries every 0.03 seconds")

    def load_and_swap(self, path: str):
        """
        파일을 읽어 'data: lon,lat' 형식에서 위경도 순서로 스왑하여 리스트로 반환
        """
        swapped = []
        try:
            with open(path, 'r') as f:
                for line in f:
                    line = line.strip()
                    # 구분자 '---'이나 빈 줄은 건너뜀
                    if not line or line.startswith('---'):
                        continue
                    # "data: lon,lat" 패턴 처리
                    if line.startswith('data:'):
                        payload = line.split(':', 1)[1].strip()
                        parts = payload.split(',')
                        if len(parts) == 2:
                            lat, lon = parts[0].strip(), parts[1].strip()
                            swapped.append(f"{lat},{lon}")
        except Exception as e:
            self.get_logger().error(f"Failed to read file '{path}': {e}")
        return swapped

    def timer_callback(self):
        # 인덱스가 범위를 벗어나면 타이머 종료
        if self._index >= len(self.data_list):
            self.get_logger().info("All data published. Shutting down timer.")
            self.timer.cancel()
            return

        msg = String()
        msg.data = self.data_list[self._index]
        self.publisher.publish(msg)
        self.get_logger().debug(f"Published: {msg.data}")
        self._index += 1


def main(args=None):
    rclpy.init(args=args)
    node = LatLonFilePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
