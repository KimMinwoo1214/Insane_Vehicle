#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class ControlCmdPublisher(Node):
    def __init__(self):
        super().__init__('control_cmd_publisher')
        # control_cmd 토픽 퍼블리셔 생성
        self.publisher = self.create_publisher(String, 'control_cmd', 10)

        # 최근 각도 저장용
        self.cone_angle = None
        self.lane_angle = None

        # cone_steering_angle, lane_steering_angle 토픽 구독
        self.create_subscription(
            Float32,
            'cone_steering_angle',
            self.cone_callback,
            10
        )
        self.create_subscription(
            Float32,
            'lane_steering_angle',
            self.lane_callback,
            10
        )

        # 초기 명령값 전송
        self.get_logger().info('ControlCmdPublisher 노드 시작, 초기 값 전송 중...')
        self.publish_command(1350, 340)

    def cone_callback(self, msg: Float32):
        self.cone_angle = msg.data
        self.process_and_publish()

    def lane_callback(self, msg: Float32):
        self.lane_angle = msg.data
        self.process_and_publish()

    def process_and_publish(self):
        # steer angle 결정: 값 0인 경우 다른 값 사용, 둘 다 있으면 cone 우선
        angle = None
        if self.cone_angle is not None and self.lane_angle is not None:
            if self.cone_angle == 0 and self.lane_angle != 0:
                angle = self.lane_angle
            elif self.lane_angle == 0 and self.cone_angle != 0:
                angle = self.cone_angle
            else:
                angle = self.cone_angle
        elif self.cone_angle is not None:
            if self.cone_angle == 0 and self.lane_angle is not None:
                angle = self.lane_angle
            else:
                angle = self.cone_angle
        elif self.lane_angle is not None:
            if self.lane_angle == 0 and self.cone_angle is not None:
                angle = self.cone_angle
            else:
                angle = self.lane_angle
        else:
            # 둘 다 없으면 초기값 유지
            self.publish_command(1350, 0)
            return

        # steering_angle (67.5~112.5) 를 PWM (800~1900) 으로 선형 매핑
        min_angle, max_angle = 67.5, 112.5
        min_pwm, max_pwm = 800, 1900
        if angle <= min_angle:
            steering_pwm = min_pwm
        elif angle >= max_angle:
            steering_pwm = max_pwm
        else:
            slope = (max_pwm - min_pwm) / (max_angle - min_angle)
            intercept = min_pwm - slope * min_angle
            steering_pwm = slope * angle + intercept
        steering_pwm = int(round(steering_pwm))

        # throttle 설정: angle 범위 벗어나면 310, 그렇지 않으면 340
        if angle < min_angle or angle > max_angle:
            throttle_pwm = 310
        else:
            throttle_pwm = 330

        self.publish_command(steering_pwm, throttle_pwm)

    def publish_command(self, steering: int, throttle: int):
        cmd_str = f"{steering},{throttle}"  # "steering,throttle"
        msg = String()
        msg.data = cmd_str
        self.publisher.publish(msg)
        self.get_logger().info(f"publishing: '{cmd_str}' on 'control_cmd'")


def main(args=None):
    rclpy.init(args=args)
    node = ControlCmdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ControlCmdPublisher 노드 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
