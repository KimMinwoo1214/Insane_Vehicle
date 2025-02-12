#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import select

# 키보드 입력과 명령 매핑
move_bindings = {
    'w': 'increase_speed',
    'x': 'decrease_speed',
    's': 'stop',
    'a': 'ccw',
    'd': 'cw',
}

MAX_SPEED = 255   # 최대 전진 속도
MIN_SPEED = -255  # 최대 후진 속도
SPEED_STEP = 50   # 속도 증가 단위

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(String, 'teleop_commands', 10)
        self.speed = 0  # 초기 속도

        self.get_logger().info("키보드 제어 활성화: W(속도 증가), X(후진 속도 증가), A(좌회전), D(우회전), S(정지)")
        self.get_logger().info("Ctrl+C를 눌러 종료")

    def get_key(self):
        """ 키보드에서 단일 키 입력을 읽어오는 함수 """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # 비동기 입력 감지
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """ 키 입력을 감지하고 속도 조절 후 ROS2 토픽으로 발행 """
        try:
            while rclpy.ok():
                key = self.get_key()
                msg = String()

                if key in move_bindings:
                    command = move_bindings[key]

                    if command == "increase_speed":
                        self.speed += SPEED_STEP
                        if self.speed > MAX_SPEED:
                            self.speed = MAX_SPEED
                        msg.data = f"forward:{self.speed}"  # 속도 값 포함
                    elif command == "decrease_speed":
                        self.speed -= SPEED_STEP
                        if self.speed < MIN_SPEED:
                            self.speed = MIN_SPEED
                        msg.data = f"backward:{abs(self.speed)}"  # 속도 값 포함
                    elif command == "stop":
                        self.speed = 0
                        msg.data = "stop"
                    elif command == "ccw":
                        msg.data = "ccw"
                    elif command == "cw":
                        msg.data = "cw"

                    self.publisher.publish(msg)
                    self.get_logger().info(f"전송됨: {msg.data}")

                elif key == '\x03':  # Ctrl+C
                    self.get_logger().info("프로그램 종료")
                    break

        except Exception as e:
            self.get_logger().error(f"오류 발생: {e}")

        finally:
            # 종료 시 정지 명령 전송
            msg = String()
            msg.data = "stop"
            self.publisher.publish(msg)
            self.get_logger().info("모터 정지 명령 전송")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = TeleopKeyboard()
    node.run()

if __name__ == "__main__":
    main()

