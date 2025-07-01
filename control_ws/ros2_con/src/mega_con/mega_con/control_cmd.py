#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class ControlCmdPublisher(Node):
    def __init__(self):
        super().__init__('control_cmd_publisher')
        # control_cmd 토픽 퍼블리셔 생성
        self.publisher = self.create_publisher(String, 'control_cmd', 10)

        # cone_steering_angle, lane_steering_angle 토픽 구독
        self.subscription_cone = self.create_subscription(
            Float32,
            'cone_steering_angle',
            self.angle_callback,
            10
        )
        self.subscription_lane = self.create_subscription(
            Float32,
            'lane_steering_angle',
            self.angle_callback,
            10
        )

        # 초기 명령값 전송
        self.get_logger().info('ControlCmdPublisher 노드 시작, 초기 값 전송 중...')
        self.publish_command(1350, 340)

    def angle_callback(self, msg: Float32):
        angle = msg.data
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

        # throttle 고정 (최대값)
        throttle_pwm = 340
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
