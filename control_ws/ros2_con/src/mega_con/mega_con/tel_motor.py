#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import select

# 상수 정의
MAX_THROTTLE = 340   # 최대 전진 속도 (340으로 확장)
MIN_THROTTLE = 0     # 정지
THROTTLE_STEP = 30    # throttle 증감 단위 (필요시 조정)

CENTER_STEERING = 1400   # 스티어링 중앙값
LEFT_MAX = 800         # 왼쪽 최대값
RIGHT_MAX = 1900         # 오른쪽 최대값
STEERING_STEP = 100      # steering 증감 단위

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        # 토픽 이름을 cmd_vel 로 변경
        self.publisher = self.create_publisher(String, 'control_cmd', 10)
        self.throttle = 0
        self.steering = CENTER_STEERING
        
        self.get_logger().info("키보드 제어 (cmd_vel) 활성화:")
        self.get_logger().info("  W: throttle 증가")
        self.get_logger().info("  S: throttle 정지 (0)")
        self.get_logger().info("  A: 왼쪽 회전 (steering 증가)")
        self.get_logger().info("  D: 오른쪽 회전 (steering 감소)")
        self.get_logger().info("  X: steering 리셋 (1400)")
        self.get_logger().info("Ctrl+C 를 눌러 종료")
    
    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                return sys.stdin.read(1)
            return ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if not key:
                    continue

                msg = String()
                # throttle 조작
                if key == 'w':
                    self.throttle = min(self.throttle + THROTTLE_STEP, MAX_THROTTLE)
                    self.get_logger().info(f"Throttle 증가 → {self.throttle}")
                elif key == 's':
                    self.throttle = MIN_THROTTLE
                    self.get_logger().info("Throttle 정지 → 0")
                # steering 조작
                elif key == 'a':
                    self.steering = min(self.steering + STEERING_STEP, LEFT_MAX)
                    self.get_logger().info(f"좌회전 → Steering {self.steering}")
                elif key == 'd':
                    self.steering = max(self.steering - STEERING_STEP, RIGHT_MAX)
                    self.get_logger().info(f"우회전 → Steering {self.steering}")
                elif key == 'x':
                    self.steering = CENTER_STEERING
                    self.get_logger().info("Steering 리셋 → 1400")
                elif key == '\x03':  # Ctrl+C
                    self.get_logger().info("프로그램 종료")
                    break
                else:
                    continue

                # 범위 내 보장된 steering, throttle 을 "steering,throttle" 형태로 발행
                msg.data = f"{self.steering},{self.throttle}"
                self.publisher.publish(msg)
                self.get_logger().info(f"발행 → {msg.data}")

        except Exception as e:
            self.get_logger().error(f"오류 발생: {e}")
        finally:
            # 종료 시 안전하게 정지
            stop = String()
            stop.data = f"{CENTER_STEERING},{MIN_THROTTLE}"
            self.publisher.publish(stop)
            self.get_logger().info("종료: 모터 정지 명령 발행")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = TeleopKeyboard()
    node.run()

if __name__ == '__main__':
    main()
