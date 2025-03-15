#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoCommander(Node):
    def __init__(self):
        super().__init__('arduino_commander')
        # teleop_commands 토픽 구독
        self.subscription = self.create_subscription(
            String,
            'teleop_commands',
            self.teleop_callback,
            10
        )
        self.subscription  # 사용되지 않는 변수 경고 제거용

        # 아두이노와의 시리얼 연결 설정
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info("아두이노 시리얼 포트 열림: /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f"아두이노 시리얼 연결 실패: {e}")
            self.ser = None

    def teleop_callback(self, msg):
        # 메시지 예: "steering,throttle" (예 "90,50")
        command_str = msg.data.strip() + "\n"  # 아두이노에서 줄바꿈을 기준으로 읽음
        self.get_logger().info(f"수신된 명령: {command_str.strip()}")

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(command_str.encode('utf-8'))
                self.get_logger().info("명령을 아두이노로 전송함")
            except Exception as e:
                self.get_logger().error(f"명령 전송 중 오류: {e}")
        else:
            self.get_logger().error("시리얼 포트가 열려있지 않습니다.")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ArduinoCommander 노드 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

