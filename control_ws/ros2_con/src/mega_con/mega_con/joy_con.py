#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial

class PS3TeleopNode(Node):
    def __init__(self):
        super().__init__('ps3_teleop_node')

        # 조이스틱 토픽 구독 (사용중인 토픽명에 맞게 수정)
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # 시리얼 포트 열기 (환경에 맞게 수정)
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)
            self.get_logger().info("Serial port opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

        # 속도/조향 스케일 (원하는 값으로 조정)
        self.max_speed = 255
        self.max_steer = 100

        # 직전 명령값 기록용(초기화)
        self.last_left_speed = None
        self.last_right_speed = None
        self.last_steer_val = None

    def joy_callback(self, msg: Joy):
        if self.ser is None:
            return  # 시리얼 포트가 열리지 않았으면 아무것도 안 함

        # PS3 컨트롤러 축 매핑(실제 축 인덱스 확인 필요)
        forward_axis = msg.axes[1]   # -1.0 ~ +1.0 (전후진)
        steering_axis = msg.axes[0]  # -1.0 ~ +1.0 (좌우)

        # 전/후진 속도 계산 (왼/오 모터 동일하게)
        left_speed = int(forward_axis * self.max_speed)
        right_speed = int(forward_axis * self.max_speed)

        # 스티어링 속도 계산
        steer_val = int(steering_axis * self.max_steer)

        # -----------------------------
        # 1) drive 명령 전송 (값이 바뀌었을 때만)
        # -----------------------------
        if (left_speed != self.last_left_speed) or (right_speed != self.last_right_speed):
            drive_cmd = f"drive: {left_speed} {right_speed}\n"
            try:
                sent_drive = self.ser.write(drive_cmd.encode('utf-8'))
                self.get_logger().info(
                    f"[DEBUG] Sent drive_cmd ({sent_drive} bytes): {drive_cmd.strip()}"
                )
            except Exception as e:
                self.get_logger().error(f"Serial write error (drive_cmd): {e}")

            # 직전값 갱신
            self.last_left_speed = left_speed
            self.last_right_speed = right_speed

        # -----------------------------
        # 2) steer 명령 전송 (값이 바뀌었을 때만)
        # -----------------------------
        if steer_val != self.last_steer_val:
            steer_cmd = f"steer: {steer_val}\n"
            try:
                sent_steer = self.ser.write(steer_cmd.encode('utf-8'))
                self.get_logger().info(
                    f"[DEBUG] Sent steer_cmd ({sent_steer} bytes): {steer_cmd.strip()}"
                )
            except Exception as e:
                self.get_logger().error(f"Serial write error (steer_cmd): {e}")

            # 직전값 갱신
            self.last_steer_val = steer_val

def main(args=None):
    rclpy.init(args=args)
    node = PS3TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

