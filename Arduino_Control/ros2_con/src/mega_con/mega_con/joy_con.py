#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy  # 조이스틱 메시지

# 속도 설정
MAX_SPEED = 255   # 최대 전진 속도
MIN_SPEED = -255  # 최대 후진 속도
SPEED_STEP = 10   # 속도 증가 단위

class TeleopJoystick(Node):
    def __init__(self):
        super().__init__('teleop_joystick')

        # 조이스틱 입력을 받는 구독자
        self.subscription = self.create_subscription(
            Joy,
            '/joy',  # ROS2에서 조이스틱 데이터가 들어오는 기본 토픽
            self.joy_callback,
            10
        )

        # 모터 명령을 전송할 퍼블리셔
        self.publisher = self.create_publisher(String, 'teleop_commands', 10)

        # 현재 속도 값
        self.speed = 0  
        self.get_logger().info("PS3 조이스틱으로 제어 시작 (좌스틱: 전진/후진, 우스틱: 좌/우회전, X 버튼: 정지)")

    def joy_callback(self, msg):
        """ 조이스틱 입력을 감지하여 속도 조절 후 ROS2 토픽으로 발행 """
        command_msg = String()

        # 아날로그 스틱 입력값 (조이스틱마다 다를 수 있음)
        linear_axis = msg.axes[1]  # 왼쪽 스틱의 위/아래 방향
        angular_axis = msg.axes[0] # 왼쪽 스틱의 좌/우 방향

        # 속도 조절 (위 방향으로 밀면 속도 증가, 아래 방향이면 감소)
        self.speed = int(linear_axis * MAX_SPEED)

        # 조향 (좌/우 방향으로 조이스틱 움직이면 회전)
        if angular_axis > 0.5:
            command_msg.data = "cw"  # 시계방향 회전
        elif angular_axis < -0.5:
            command_msg.data = "ccw"  # 반시계방향 회전
        else:
            command_msg.data = f"forward:{self.speed}" if self.speed > 0 else f"backward:{abs(self.speed)}"

        # X 버튼 (`buttons[1]`)을 누르면 정지
        if msg.buttons[1] == 1:
            command_msg.data = "stop"
            self.speed = 0

        self.publisher.publish(command_msg)
        self.get_logger().info(f"전송됨: {command_msg.data}")

def main():
    rclpy.init()
    node = TeleopJoystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
