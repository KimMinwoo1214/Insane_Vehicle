#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import serial.serialutil

class ArduinoCommander(Node):
    def __init__(self):
        super().__init__('arduino_commander')

        # control_cmd 토픽 구독
        self.subscription = self.create_subscription(
            String,
            'control_cmd',
            self.teleop_callback,
            10
        )
        self.subscription  # 사용되지 않는 변수 경고 제거용

        # 마지막으로 전송된 명령 저장
        self.last_command = None

        # 시리얼 포트 설정
        self.port = '/dev/ttyACM0'
        self.baudrate = 115200

        # 초기 시리얼 연결 시도
        self.ser = None
        self._open_serial()

        # 시리얼 상태 주기적 점검 (0.3초마다)
        self.create_timer(0.3, self._check_and_reconnect)

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"아두이노 시리얼 연결됨: {self.port}@{self.baudrate}")
            # 재연결 직후, 마지막 명령이 있으면 재전송
            if self.last_command:
                self._send_to_arduino(self.last_command)
        except Exception as e:
            self.get_logger().warn(f"시리얼 연결 실패: {e}")
            self.ser = None

    def _check_and_reconnect(self):
        # 이미 열려 있으면 아무 동작도 하지 않음
        if self.ser and self.ser.is_open:
            return
        # 닫혀 있으면 재연결 시도
        self.get_logger().info("시리얼 연결 재시도 중...")
        self._open_serial()

    def teleop_callback(self, msg: String):
        command_str = msg.data.strip() + "\n"
        self.get_logger().info(f"수신된 명령: {command_str.strip()}")
        # 마지막 명령 기록
        self.last_command = command_str
        self._send_to_arduino(command_str)

    def _send_to_arduino(self, cmd: str):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("시리얼 포트가 열려 있지 않아 명령 전송을 건너뜁니다")
            return
        try:
            self.ser.write(cmd.encode('utf-8'))
            self.get_logger().info("명령을 아두이노로 전송함")
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f"명령 전송 중 시리얼 에러: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
