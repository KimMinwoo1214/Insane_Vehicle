#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Int32, String

# 데드존 함수
DEADZONE = 0.1
def apply_deadzone(value: float) -> float:
    if abs(value) < DEADZONE:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    scaled = (abs(value) - DEADZONE) / (1.0 - DEADZONE)
    return sign * scaled

class ControlCmdPublisher(Node):
    def __init__(self):
        super().__init__('control_cmd_publisher')

        # 파라미터 선언/취득
        self.declare_parameter('emergency_topic', '/emergency')
        self.declare_parameter('cone_angle_topic', '/cone_steering_angle')
        self.declare_parameter('lane_angle_topic', '/lane_steering_angle')
        self.declare_parameter('control_cmd_topic', '/control_cmd')
        self.declare_parameter('publish_rate', 10.0)

        self.emergency_topic = self.get_parameter('emergency_topic').value
        cone_topic    = self.get_parameter('cone_angle_topic').value
        lane_topic    = self.get_parameter('lane_angle_topic').value
        control_topic = self.get_parameter('control_cmd_topic').value
        rate_hz       = self.get_parameter('publish_rate').value

        # 퍼블리셔 & 구독
        self.pub_cmd = self.create_publisher(String, control_topic, 10)
        self.create_subscription(Float32, cone_topic, self._cone_cb, 10)
        self.create_subscription(Float32, lane_topic, self._lane_cb, 10)
        self.create_subscription(Int32,   self.emergency_topic, self._emergency_cb, 10)
        self.create_subscription(Joy,     'joy', self._joy_cb, 10)

        # 내부 상태
        self._last_steering     = 2100
        self._last_throttle     = 550
        self._last_sent_cmd     = None
        self._last_sent_raw_ang = None
        self._last_mode         = "Unknown"

        self.emergency_flag = False
        self.cone_angle     = None
        self.lane_angle     = None

        # 조이스틱 제어 관련
        self.joy_axes = [0.0, 0.0]   # [steer, throttle]
        self.joy_mode = False        # False=자동, True=직접

        # 버튼 연속 눌림 카운터
        self.btn5_count = 0
        self.btn4_count = 0
        self.btn3_count = 0
        self.BUTTON_HOLD_THRESHOLD = 5  # 연속 콜백 수

        # 주기 타이머
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

    def _joy_cb(self, msg: Joy):
        # 버튼 상태를 list로
        buttons = list(msg.buttons)
        n = len(buttons)
        idx5 = n - 5
        idx4 = n - 4
        idx3 = n - 3

        # 5번째 버튼 카운트
        if buttons[idx5] == 1:
            self.btn5_count += 1
        else:
            self.btn5_count = 0
        if self.btn5_count == self.BUTTON_HOLD_THRESHOLD:
            # 즉시 throttle 컷오프
            self._last_throttle = 0
            cmd = f"{self._last_steering},{self._last_throttle}"
            self.pub_cmd.publish(String(data=cmd))
            self.get_logger().info(f"[BUTTON_5 x{self.BUTTON_HOLD_THRESHOLD}] 즉시 퍼블리시 → '{cmd}'")

        # 4번째 버튼 카운트
        if buttons[idx4] == 1:
            self.btn4_count += 1
        else:
            self.btn4_count = 0
        if self.btn4_count == self.BUTTON_HOLD_THRESHOLD:
            self.joy_mode = True
            self.get_logger().info(f"[BUTTON_4 x{self.BUTTON_HOLD_THRESHOLD}] JOYSTICK MODE ON")

        # 3번째 버튼 카운트
        if buttons[idx3] == 1:
            self.btn3_count += 1
        else:
            self.btn3_count = 0
        if self.btn3_count == self.BUTTON_HOLD_THRESHOLD:
            self.joy_mode = False
            self.get_logger().info(f"[BUTTON_3 x{self.BUTTON_HOLD_THRESHOLD}] AUTO MODE ON")

        # 데드존 적용 후 축 값 저장
        raw_steer    = msg.axes[0]
        raw_throttle = msg.axes[1]
        steer   = apply_deadzone(raw_steer)
        throttle= apply_deadzone(raw_throttle)
        self.joy_axes = [steer, throttle]

    def _compute_command(self):
        # 1) 조이스틱 직접 제어 모드
        if self.joy_mode:
            steer_axis    = self.joy_axes[0]
            throttle_axis = self.joy_axes[1]

            # steer 계산 (+1→min_a, -1→max_a)
            min_a, max_a = 67.5, 112.5
            angle = ((-steer_axis + 1) / 2) * (max_a - min_a) + min_a
            angle = round(angle)
            min_p, max_p = 1350, 2800
            if angle <= min_a:
                sp = min_p
            elif angle >= max_a:
                sp = max_p
            else:
                slope = (max_p - min_p) / (max_a - min_a)
                sp = int(round(slope * angle + (min_p - slope * min_a)))
            self._last_steering = sp

            # throttle 계산 (>0→axis*550, ≤0→0)
            if throttle_axis > 0:
                self._last_throttle = int(round(throttle_axis * 550))
            else:
                self._last_throttle = 0

            self._last_mode = "Joystick"
            return

        # 2) 자동(cone/lane) 제어 기존 로직
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

        # throttle 계산
        if self.emergency_flag:
            tp = 0
        else:
            if angle is None:
                tp = self._last_throttle
            else:
                tp = 500 if (angle < 67.5 or angle > 112.5) else 550
        self._last_throttle = tp

    def _timer_publish(self):
        # raw_angle 판단
        raw_angle = None
        if self.cone_angle is not None or self.lane_angle is not None:
            if self.cone_angle == 0 and self.lane_angle is not None:
                raw_angle = self.lane_angle
            else:
                raw_angle = self.cone_angle or self.lane_angle

        self._compute_command()
        cmd = f"{self._last_steering},{self._last_throttle}"

        # 퍼블리시 조건
        send = False
        if raw_angle is not None and not self.joy_mode:
            if self._last_sent_raw_ang is None or abs(raw_angle - self._last_sent_raw_ang) >= 0.2:
                send = True
                self._last_sent_raw_ang = raw_angle
        else:
            if cmd != self._last_sent_cmd:
                send = True

        if send:
            self.pub_cmd.publish(String(data=cmd))
            self.get_logger().info(f"publishing: '{cmd}' [mode={self._last_mode}]")
            self._last_sent_cmd = cmd
        else:
            self.get_logger().debug("변화 없음, 전송 생략")

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

