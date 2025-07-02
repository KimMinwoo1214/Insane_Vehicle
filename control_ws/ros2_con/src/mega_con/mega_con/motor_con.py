#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import serial.serialutil
import time

class ArduinoCommander(Node):
    def __init__(self):
        super().__init__('arduino_commander')

        # 파라미터 선언
        self.declare_parameter('control_cmd_topic', '/control_cmd')
        self.declare_parameter('publish_rate', 5.0)       # Hz 명령 전송 주기
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_check_rate', 0.3)  # 초

        # 파라미터 취득
        topic       = self.get_parameter('control_cmd_topic').value
        rate_hz     = self.get_parameter('publish_rate').value
        self.port   = self.get_parameter('serial_port').value
        self.baud   = self.get_parameter('baudrate').value
        check_rate  = self.get_parameter('serial_check_rate').value

        # 내부 상태
        self.last_command = None
        self.send_count = 0
        self.reconnect_count = 0

        # 구독: control_cmd 토픽
        self.get_logger().info(f"[INIT] Subscribing to topic: {topic}")
        self.subscription = self.create_subscription(
            String,
            topic,
            self.teleop_callback,
            1  # QoS depth
        )

        # 시리얼 초기화 및 재연결 타이머
        self.ser = None
        self._open_serial()
        self.create_timer(check_rate, self._check_and_reconnect)
        # 명령 전송 타이머
        self.create_timer(1.0 / rate_hz, self._timer_publish)
        self.get_logger().info(f"[INIT] Timers: publish_rate={rate_hz}Hz, check_rate={check_rate}s")

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"[SERIAL] Connected: {self.port}@{self.baud}")
            # 재연결 후 마지막 명령 재전송
            if self.last_command:
                self.get_logger().debug("[SERIAL] Resending last command after reconnect")
                self._send_to_arduino(self.last_command)
        except Exception as e:
            self.get_logger().warn(f"[SERIAL] Connection failed: {e}")
            self.ser = None

    def _check_and_reconnect(self):
        if self.ser and self.ser.is_open:
            self.get_logger().debug("[SERIAL] Port alive")
            return
        self.reconnect_count += 1
        self.get_logger().warn(f"[SERIAL] Port closed, reconnect attempt #{self.reconnect_count}")
        self._open_serial()

    def teleop_callback(self, msg: String):
        cmd = msg.data.strip() + "\n"
        self.last_command = cmd
        self.get_logger().info(f"[CALLBACK] Received command: {cmd.strip()}")
        self._send_to_arduino(cmd)

    def _timer_publish(self):
        if not self.last_command:
            self.get_logger().debug("[TIMER] No command to send yet")
            return
        self._send_to_arduino(self.last_command)

    def _send_to_arduino(self, cmd: str):
        if not self.ser or not self.ser.is_open:
            self.get_logger().error("[SEND] Serial port closed, skipping send")
            return
        try:
            start = time.time()
            n = self.ser.write(cmd.encode('utf-8'))
            self.ser.flush()
            end = time.time()
            self.send_count += 1
            self.get_logger().debug(
                f"[SEND] #{self.send_count} Sent {n} bytes: '{cmd.strip()}' "
                f"(took {((end-start)*1000):.2f} ms)"
            )
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f"[SEND] SerialException: {e}")
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
