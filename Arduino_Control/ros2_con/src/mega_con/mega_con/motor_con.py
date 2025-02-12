#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # 아두이노와 연결할 시리얼 포트 설정 (포트 확인 후 수정 필요)
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

        # ROS2에서 키보드 명령을 받기 위한 Subscriber 생성
        self.subscription = self.create_subscription(
            String,
            'teleop_commands',
            self.command_callback,
            10
        )

        self.get_logger().info("Serial Bridge Node Initialized.")

    def command_callback(self, msg):
        command = msg.data.strip()
        self.get_logger().info(f"Received Command: {command}")

        # 아두이노로 명령어 전송
        self.serial_port.write(f"{command}\n".encode())

    def destroy_node(self):
        self.serial_port.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

