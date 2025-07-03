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
        self._last_steering = 1400            # 초기 스티어링 PWM
        self._last_throttle = 350             # 초기 스로틀 PWM
        self._last_sent_cmd = None            # 마지막으로 전송한 cmd 문자열
        self._last_sent_raw_angle = None      # 마지막 전송 시 원본 각도
        self.emergency_flag = False
        self.cone_angle = None
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
        # 1) 스티어링 각도 결정 (cone 우선)
        angle = None
        if self.cone_angle is not None or self.lane_angle is not None:
            if self.cone_angle is not None and self.lane_angle is not None:
                angle = self.lane_angle if self.cone_angle == 0 else self.cone_angle
            else:
                angle = self.cone_angle or self.lane_angle

        # 2) 1도 단위 양자화 적용
        if angle is not None:
            angle = round(angle)

            # 3) steering -> PWM 매핑
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

        # 4) throttle 결정
        if self.emergency_flag:
            tp = 0
        else:
            if angle is None:
                tp = self._last_throttle
            else:
                tp = 310 if (angle < min_a or angle > max_a) else 350
        self._last_throttle = tp

    def _timer_publish(self):
        # 0) 원본 각도 취득 (cone 또는 lane)
        raw_angle = None
        if self.cone_angle is not None or self.lane_angle is not None:
            raw_angle = self.lane_angle if (self.cone_angle == 0 and self.lane_angle is not None) else (self.cone_angle or self.lane_angle)

        # 최신 센서/플래그 기준으로 명령 계산
        self._compute_command()
        cmd = f"{self._last_steering},{self._last_throttle}"

        # 발행 여부 결정
        send = False
        if raw_angle is not None:
            # 원본 각도 변화량 비교 (1도 이상 변화 시 전송)
            if self._last_sent_raw_angle is None or abs(raw_angle - self._last_sent_raw_angle) >= 1.0:
                send = True
                self._last_sent_raw_angle = raw_angle
        else:
            # 각도 정보 없으면 기존 문자열 비교
            if cmd != self._last_sent_cmd:
                send = True

        # 실제 발행
        if send:
            msg = String()
            msg.data = cmd
            self.pub_cmd.publish(msg)
            self.get_logger().info(
                f"publishing: '{cmd}' (raw Δ={'N/A' if self._last_sent_raw_angle is None else raw_angle - self._last_sent_raw_angle:.1f}°)"
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
