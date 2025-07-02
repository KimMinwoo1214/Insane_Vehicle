#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String

class ControlCmdPublisher(Node):
    def __init__(self):
        super().__init__('control_cmd_publisher')

        # 파라미터 선언
        self.declare_parameter('emergency_topic', '/emergency')
        self.declare_parameter('cone_angle_topic', 'cone_steering_angle')
        self.declare_parameter('lane_angle_topic', 'lane_steering_angle')
        self.declare_parameter('control_cmd_topic', 'control_cmd')
        self.declare_parameter('publish_rate', 2.0)  # Hz로 명령 전송 주기

        # 파라미터 취득
        self.emergency_topic   = self.get_parameter('emergency_topic').value
        cone_topic             = self.get_parameter('cone_angle_topic').value
        lane_topic             = self.get_parameter('lane_angle_topic').value
        control_topic          = self.get_parameter('control_cmd_topic').value
        rate_hz                = self.get_parameter('publish_rate').value

        # 퍼블리셔 (control_cmd)
        self.pub_cmd = self.create_publisher(String, control_topic, 10)

        # 최신 명령 저장용 변수
        self._last_steering = 1400  # 초기 스티어링 PWM
        self._last_throttle = 350   # 초기 스로틀 PWM
        self.emergency_flag = False
        self.cone_angle = None
        self.lane_angle = None

        # 구독: steering 토픽 (값이 들어올 때마다 저장만)
        self.create_subscription(Float32, cone_topic, self._cone_cb, 10)
        self.create_subscription(Float32, lane_topic, self._lane_cb, 10)

        # 구독: emergency 토픽 (플래그 갱신)
        self.create_subscription(Int32, self.emergency_topic, self._emergency_cb, 10)

        # 주기 발행 타이머 (1/rate_hz 초마다)
        self.create_timer(1.0 / rate_hz, self._timer_publish)

        self.get_logger().info(f'ControlCmdPublisher 시작 (publish_rate={rate_hz}Hz)')

    def _emergency_cb(self, msg: Int32):
        # 1: 비상, 0: 해제
        prev = self.emergency_flag
        self.emergency_flag = (msg.data == 1)
        if self.emergency_flag != prev:
            self.get_logger().info(f"[EMERGENCY] flag -> {self.emergency_flag}")

    def _cone_cb(self, msg: Float32):
        self.cone_angle = msg.data

    def _lane_cb(self, msg: Float32):
        self.lane_angle = msg.data

    def _compute_command(self):
        # 1) 스티어링 각도 결정 (cone 우선)
        angle = None
        if self.cone_angle is not None or self.lane_angle is not None:
            if self.cone_angle is not None and self.lane_angle is not None:
                # cone_angle==0 이면 lane 적용
                angle = self.lane_angle if self.cone_angle == 0 else self.cone_angle
            else:
                angle = self.cone_angle or self.lane_angle

        # 2) steering → PWM 매핑
        if angle is not None:
            min_a, max_a = 67.5, 112.5
            min_p, max_p = 800, 1900
            if angle <= min_a:
                sp = min_p
            elif angle >= max_a:
                sp = max_p
            else:
                slope = (max_p - min_p) / (max_a - min_a)
                sp = int(round(slope * angle + (min_p - slope * min_a)))
            self._last_steering = sp

        # 3) throttle 결정
        if self.emergency_flag:
            tp = 0
        else:
            # angle이 None이면 이전 tp 유지
            if angle is None:
                tp = self._last_throttle
            else:
                tp = 310 if (angle < min_a or angle > max_a) else 350
        self._last_throttle = tp

    def _timer_publish(self):
        # 최신 센서/플래그 기준으로 명령 계산 후 발행
        self._compute_command()
        cmd = f"{self._last_steering},{self._last_throttle}"
        msg = String()
        msg.data = cmd
        self.pub_cmd.publish(msg)
        self.get_logger().info(f"publishing: '{cmd}'")

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
