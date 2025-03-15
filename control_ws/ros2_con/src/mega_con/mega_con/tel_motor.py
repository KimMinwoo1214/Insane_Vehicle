#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import select

# 상수 정의
MAX_THROTTLE = 30    # 최대 전진 속도
MIN_THROTTLE = 0      # 정지
THROTTLE_STEP = 5    # throttle 증감 단위

CENTER_STEERING = 90  # 스티어링 중앙값
LEFT_MAX = 123        # 왼쪽 최대값 (여러번 'a'를 눌러야 도달)
RIGHT_MAX = 54        # 오른쪽 최대값 (여러번 'd'를 눌러야 도달)
STEERING_STEP = 8     # steering 증감 단위

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(String, 'teleop_commands', 10)
        self.throttle = 0
        self.steering = CENTER_STEERING
        
        self.get_logger().info("키보드 제어 활성화:")
        self.get_logger().info("  W: throttle 증가")
        self.get_logger().info("  S: throttle 정지 (0)")
        self.get_logger().info("  A: 왼쪽 회전 (steering 증가)")
        self.get_logger().info("  D: 오른쪽 회전 (steering 감소)")
        self.get_logger().info("  X: steering 리셋 (90)")
        self.get_logger().info("Ctrl+C를 눌러 종료")
    
    def get_key(self):
        """ 키보드에서 단일 키 입력을 읽어옵니다 """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                msg = String()

                if key == 'w':
                    # throttle 증가
                    self.throttle += THROTTLE_STEP
                    if self.throttle > MAX_THROTTLE:
                        self.throttle = MAX_THROTTLE
                    msg.data = f"{self.steering},{self.throttle}"
                    self.get_logger().info(f"Throttle 증가: {self.throttle}")
                elif key == 's':
                    # throttle 정지
                    self.throttle = 0
                    msg.data = f"{self.steering},{self.throttle}"
                    self.get_logger().info("Throttle 정지: 0")
                elif key == 'a':
                    # 왼쪽 회전: steering 값 증가 (최대 LEFT_MAX)
                    self.steering += STEERING_STEP
                    if self.steering > LEFT_MAX:
                        self.steering = LEFT_MAX
                    msg.data = f"{self.steering},{self.throttle}"
                    self.get_logger().info(f"좌회전: Steering {self.steering}")
                elif key == 'd':
                    # 오른쪽 회전: steering 값 감소 (최소 RIGHT_MAX)
                    self.steering -= STEERING_STEP
                    if self.steering < RIGHT_MAX:
                        self.steering = RIGHT_MAX
                    msg.data = f"{self.steering},{self.throttle}"
                    self.get_logger().info(f"우회전: Steering {self.steering}")
                elif key == 'x':
                    # 스티어링 리셋: 중앙값 90으로 복원
                    self.steering = CENTER_STEERING
                    msg.data = f"{self.steering},{self.throttle}"
                    self.get_logger().info("스티어링 리셋: 90")
                elif key == '\x03':  # Ctrl+C
                    self.get_logger().info("프로그램 종료")
                    break
                else:
                    continue

                # 명령 메시지 발행
                self.publisher.publish(msg)
                self.get_logger().info(f"명령 전송: {msg.data}")
                
        except Exception as e:
            self.get_logger().error(f"오류 발생: {e}")
        finally:
            # 종료 시 throttle 0 (정지) 명령 전송
            msg = String()
            msg.data = f"{self.steering},0"
            self.publisher.publish(msg)
            self.get_logger().info("종료: 모터 정지 명령 전송")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = TeleopKeyboard()
    node.run()

if __name__ == '__main__':
    main()

