#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, serial.serialutil
import threading
import queue
import time

class ArduinoCommander(Node):
    def __init__(self):
        super().__init__('arduino_commander')
        # 파라미터
        self.declare_parameter('control_cmd_topic', '/control_cmd')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        topic      = self.get_parameter('control_cmd_topic').value
        self.port  = self.get_parameter('serial_port').value
        self.baud  = self.get_parameter('baudrate').value

        # 구독 (명령 큐에 저장만)
        self.cmd_queue = queue.Queue(maxsize=5)
        self.create_subscription(String, topic, self.teleop_callback, 1)
        self.get_logger().info(f"Subscribing to {topic}")

        # 시리얼 연결 준비
        self.ser = None
        self._open_serial()

        # 시리얼 전송 전용 쓰레드
        self.worker = threading.Thread(target=self._serial_worker, daemon=True)
        self.worker.start()

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"[SERIAL] Connected at {self.port}@{self.baud}")
        except Exception as e:
            self.get_logger().warn(f"[SERIAL] Connect failed: {e}")
            self.ser = None

    def teleop_callback(self, msg: String):
        cmd = msg.data.strip() + "\n"
        try:
            self.cmd_queue.put_nowait(cmd)
            self.get_logger().debug(f"[CALLBACK] Cmd queued: {cmd.strip()}")
        except queue.Full:
            self.get_logger().warn("[CALLBACK] Cmd queue full, dropping")

    def _serial_worker(self):
        while rclpy.ok():
            try:
                cmd = self.cmd_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            # 연결 확인/재연결
            if not (self.ser and self.ser.is_open):
                self._open_serial()
                time.sleep(0.1)
                if not (self.ser and self.ser.is_open):
                    self.get_logger().error("[WORKER] Still no serial, skip send")
                    continue

            # 실제 쓰기
            try:
                start = time.time()
                n = self.ser.write(cmd.encode('utf-8'))
                # flush 제거 → 블로킹 시간 단축
                elapsed = (time.time() - start) * 1000
                self.get_logger().debug(f"[WORKER] Sent {n} bytes '{cmd.strip()}' ({elapsed:.1f} ms)")
            except serial.serialutil.SerialException as e:
                self.get_logger().error(f"[WORKER] SerialException: {e}")
                try: self.ser.close()
                except: pass
                self.ser = None

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommander()
    # 멀티스레드 익스큐터로 spin
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
