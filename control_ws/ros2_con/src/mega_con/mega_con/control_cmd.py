#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String

class ControlCmdPublisher(Node):
    def __init__(self):
        super().__init__('control_cmd_publisher')

        # 파라미터 선언
        self.declare_parameter('emergency_topic', '/emergency')
        self.declare_parameter('cone_angle_topic', '/cone_steering_angle')
        self.declare_parameter('lane_angle_topic', '/lane_steering_angle')
        self.declare_parameter('control_cmd_topic', '/control_cmd')
        self.declare_parameter('publish_rate', 10.0)  # Hz로 명령 전송 주기

        # 파라미터 취득
        self.emergency_topic = self.get_parameter('emergency_topic').value
        cone_topic = self.get_parameter('cone_angle_topic').value
        lane_topic = self.get_parameter('lane_angle_topic').value
        control_topic = self.get_parameter('control_cmd_topic').value
        rate_hz = self.get_parameter('publish_rate').value

        # 퍼블리셔 (control_cmd)
        self.pub_cmd = self.create_publisher(String, control_topic, 10)

        # 상태 저장용 변수
        self._last_steering = 2100
        self._last_throttle = 550
        self._last_sent_cmd = None
        self._last_sent_raw_angle = None
        self._last_mode = "Unknown"
        self.emergency_flag = False
        self.cone_angle = 0
        self.lane_angle = None

        # 구독: steering 토픽
        self.create_subscription(Float32, cone_topic, self._cone_cb, 10)
        self.create_subscription(Float32, lane_topic, self._lane_cb, 10)

        # 구독: emergency 토픽
        self.create_subscription(Int32, self.emergency_topic, self._emergency_cb, 10)

        # 주기 발행 타이머
        self.create_timer(1.0 / rate_hz, self._timer_publish)

        self.get_logger().info(f'ControlCmdPublisher 시작 (publish_rate={rate_hz}Hz)')

    def _emergency_cb(self, msg: Int32):
        prev = self.emergency_flag
        self.emergency_flag = (msg.data == 1)
        if self.emergency_flag != prev:
            self.get_logger().info(f"[EMERGENCY] flag -> {self.emergency_flag}")

    def _cone_cb(self, msg: Float32):
        self.cone_angle = msg.data

    def _lane_cb(self, msg: Float32):
        self.lane_angle = msg.data

    def _compute_command(self):
        angle = None
        if self.cone_angle is not None or self.lane_angle is not None:
            if self.cone_angle is not None and self.lane_angle is not None:
                if self.cone_angle == 0:
                    angle = self.lane_angle
                    self._last_mode = "Lane"
                else:
                    angle = self.cone_angle
                    self._last_mode = "Cone"
            else:
                angle = self.cone_angle or self.lane_angle
                self._last_mode = "Cone" if self.cone_angle is not None else "Lane"
        else:
            self._last_mode = "Unknown"

        if angle is not None:
            angle = round(angle)

            min_a, max_a = 67.5, 112.5
            min_p, max_p = 1350, 2800
            if angle <= min_a:
                sp = min_p
            elif angle >= max_a:
                sp = max_p
            else:
                slope = (max_p - min_p) / (max_a - min_a)
                sp = int(round(slope * angle + (min_p - slope * min_a)))
            self._last_steering = sp

        if self.emergency_flag:
            tp = 0
        else:
            if angle is None:
                tp = self._last_throttle
            else:
                tp = 500 if (angle < 67.5 or angle > 112.5) else 550
        self._last_throttle = tp

    def _timer_publish(self):
        raw_angle = None
        if self.cone_angle is not None or self.lane_angle is not None:
            if self.cone_angle == 0 and self.lane_angle is not None:
                raw_angle = self.lane_angle
            else:
                raw_angle = self.cone_angle or self.lane_angle

        self._compute_command()
        cmd = f"{self._last_steering},{self._last_throttle}"

        send = False
        if raw_angle is not None:
            if self._last_sent_raw_angle is None or abs(raw_angle - self._last_sent_raw_angle) >= 0.2:
                send = True
                self._last_sent_raw_angle = raw_angle
        else:
            if cmd != self._last_sent_cmd:
                send = True

        if send:
            msg = String()
            msg.data = cmd
            self.pub_cmd.publish(msg)
            delta_str = (
                "N/A"
                if self._last_sent_raw_angle is None
                else f"{raw_angle - self._last_sent_raw_angle:.1f}"
            )
            self.get_logger().info(
                f"publishing: '{cmd}' [mode={self._last_mode}] (raw Δ={delta_str}°)"
            )
            self._last_sent_cmd = cmd
        else:
            self.get_logger().debug("Δ각도 < 1°이거나 명령 변화 없어 전송 생략")


def main(args=None):
    rclpy.init(args=args)
    node = ControlCmdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
