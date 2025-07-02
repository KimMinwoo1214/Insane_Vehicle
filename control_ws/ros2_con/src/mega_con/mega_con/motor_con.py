#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import serial.serialutil

class ArduinoCommander(Node):
    def __init__(self):
        super().__init__('arduino_commander')

        # 파라미터 선언
        self.declare_parameter('control_cmd_topic', '/control_cmd')
        self.declare_parameter('publish_rate', 5.0)       # Hz로 명령 전송 주기 (기본 5Hz)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 500000)
        self.declare_parameter('serial_check_rate', 0.3)  # 시리얼 재연결 체크 주기 (초)

        # 파라미터 취득
        topic       = self.get_parameter('control_cmd_topic').value
        rate_hz     = self.get_parameter('publish_rate').value
        self.port   = self.get_parameter('serial_port').value
        self.baud   = self.get_parameter('baudrate').value
        check_rate  = self.get_parameter('serial_check_rate').value

        # 구독: control_cmd 토픽
        self.get_logger().info(f"구독 시작: {topic}")
        self.subscription = self.create_subscription(
            String,
            topic,
            self.teleop_callback,
            1  # QoS depth 최소화
        )
        # 마지막 명령 저장
        self.last_command = None

        # 시리얼 초기화 및 재연결 타이머
        self.ser = None
        self._open_serial()
        self.create_timer(check_rate, self._check_and_reconnect)
        # 명령 전송 타이머
        self.create_timer(1.0 / rate_hz, self._timer_publish)

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"아두이노 시리얼 연결됨: {self.port}@{self.baud}")
            # 재연결 직후 마지막 명령 즉시 전송
            if self.last_command:
                self._send_to_arduino(self.last_command)
        except Exception as e:
            self.get_logger().warn(f"시리얼 연결 실패: {e}")
            self.ser = None

    def _check_and_reconnect(self):
        if self.ser and self.ser.is_open:
            return
        self.get_logger().info("시리얼 연결 재시도 중...")
        self._open_serial()

    def teleop_callback(self, msg: String):
        # 토픽 수신 시 즉시 저장 + 전송
        cmd = msg.data.strip() + "\n"
        self.last_command = cmd
        self.get_logger().info(f"수신된 명령 업데이트: {cmd.strip()}")
        self._send_to_arduino(cmd)

    def _timer_publish(self):
        # 주기적 전송: 새로 수신된 last_command를 반복 전송
        if not self.last_command:
            return
        self._send_to_arduino(self.last_command)

    def _send_to_arduino(self, cmd: str):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("시리얼 포트가 닫혀 있어 명령 전송 생략")
            return
        try:
            self.ser.write(cmd.encode('utf-8'))
            self.ser.flush()
            self.get_logger().debug(f"명령 전송: {cmd.strip()}")
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f"시리얼 전송 에러: {e}")
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
