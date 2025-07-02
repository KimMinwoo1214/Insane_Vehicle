#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import serial.serialutil
import threading
import queue
import time

class ArduinoCommander(Node):
    def __init__(self):
        super().__init__('arduino_commander')

        # === 파라미터 ===
        self.declare_parameter('control_cmd_topic', '/control_cmd')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        topic      = self.get_parameter('control_cmd_topic').value
        self.port  = self.get_parameter('serial_port').value
        self.baud  = self.get_parameter('baudrate').value

        # === 최신 명령 큐 (최대 1개) ===
        self.cmd_queue = queue.Queue(maxsize=1)

        # === 구독 ===
        self.create_subscription(String, topic, self.teleop_callback, 1)
        self.get_logger().info(f"[INIT] Subscribing to {topic}")

        # === 시리얼 연결 ===
        self.ser = None
        self._open_serial()

        # === 시리얼 워커 스레드 ===
        self.running = True
        self.worker = threading.Thread(target=self._serial_worker, daemon=True)
        self.worker.start()

    def _open_serial(self):
        try:
            # Non-blocking 모드로 timeout=0 추천
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.get_logger().info(f"[SERIAL] Connected at {self.port}@{self.baud}")
        except Exception as e:
            self.get_logger().warn(f"[SERIAL] Connect failed: {e}")
            self.ser = None

    def teleop_callback(self, msg: String):
        cmd = msg.data.strip() + "\n"

        # === 큐 비우고 최신 명령만 유지 ===
        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
            except queue.Empty:
                break

        try:
            self.cmd_queue.put_nowait(cmd)
            self.get_logger().debug(f"[CALLBACK] Cmd queued: {cmd.strip()}")
        except queue.Full:
            self.get_logger().warn("[CALLBACK] Cmd queue full, dropping")

    def _serial_worker(self):
        while self.running and rclpy.ok():
            try:
                cmd = self.cmd_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            # === 시리얼 연결 확인 ===
            if not (self.ser and self.ser.is_open):
                self._open_serial()
                time.sleep(0.1)
                if not (self.ser and self.ser.is_open):
                    self.get_logger().error("[WORKER] No serial, skip send")
                    continue

            # === 실제 전송 ===
            try:
                start = time.time()
                n = self.ser.write(cmd.encode('utf-8'))
                elapsed = (time.time() - start) * 1000
                if elapsed > 1000:
                    self.get_logger().warn(f"[WORKER] Write took unusually long: {elapsed:.1f} ms")
                else:
                    self.get_logger().info(
                        f"[WORKER] Sent {n} bytes '{cmd.strip()}' ({elapsed:.1f} ms)"
                    )
            except serial.serialutil.SerialException as e:
                self.get_logger().error(f"[WORKER] SerialException: {e}")
                try:
                    self.ser.close()
                except:
                    pass
                self.ser = None

    def destroy_node(self):
        super().destroy_node()
        self.get_logger().info("[SHUTDOWN] Clearing queue & stopping worker")
        self.running = False
        while not self.cmd_queue.empty():
            self.cmd_queue.get()
        self.worker.join(timeout=2.0)
        self.get_logger().info("[SHUTDOWN] ArduinoCommander stopped")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommander()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("[MAIN] Shutting down executor")
        executor.shutdown()

        node.get_logger().info("[MAIN] Destroying node")
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

