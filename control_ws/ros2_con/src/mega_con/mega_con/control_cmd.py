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

        # 파라미터 취득
        self.emergency_topic = self.get_parameter('emergency_topic').value
        cone_topic            = self.get_parameter('cone_angle_topic').value
        lane_topic            = self.get_parameter('lane_angle_topic').value
        control_topic         = self.get_parameter('control_cmd_topic').value

        # 퍼블리셔 (control_cmd)
        self.pub_cmd = self.create_publisher(String, control_topic, 10)

        # 스티어링 값 저장
        self.cone_angle = None
        self.lane_angle = None
        self.last_steering_pwm = 1400  # 초기 스티어링 값

        # 구독: steering
        self.create_subscription(Float32, cone_topic, self.cone_callback, 10)
        self.create_subscription(Float32, lane_topic, self.lane_callback, 10)

        # 구독: emergency
        self.emergency_flag = False
        self.create_subscription(Int32, self.emergency_topic,
                                 self.emergency_callback, 10)

        # 초기 명령
        self.get_logger().info('ControlCmdPublisher 시작')
        self.publish_command(self.last_steering_pwm, 350)

    def emergency_callback(self, msg: Int32):
        if msg.data == 1:
            self.emergency_flag = True	
        else:
            self.emergency_flag = False

    def cone_callback(self, msg: Float32):
        self.cone_angle = msg.data
        self.process_and_publish()

    def lane_callback(self, msg: Float32):
        self.lane_angle = msg.data
        self.process_and_publish()

    def process_and_publish(self):
        # 스티어링 각도 결정 (cone 우선)
        angle = None
        if self.cone_angle is not None or self.lane_angle is not None:
            if self.cone_angle is not None and self.lane_angle is not None:
                angle = (self.lane_angle
                         if self.cone_angle == 0 else self.cone_angle)
            else:
                angle = self.cone_angle or self.lane_angle
        else:
            # 둘 다 없으면 초기값
            self.publish_command(self.last_steering_pwm, 0)
            return

        # steering → PWM 매핑
        min_a, max_a = 67.5, 112.5
        min_p, max_p = 800, 1900
        if angle <= min_a:
            sp = min_p
        elif angle >= max_a:
            sp = max_p
        else:
            slope = (max_p - min_p) / (max_a - min_a)
            sp = slope * angle + (min_p - slope * min_a)
        sp = int(round(sp))
        self.last_steering_pwm = sp

        # emergency 플래그가 켜져 있으면 throttle=0
        if self.emergency_flag == 1:
            tp = 50
        else:
            # 정상 구간: angle 범위 벗어나면 310, 아니면 330
            tp = 310 if (angle < min_a or angle > max_a) else 350

        self.publish_command(sp, tp)

    def publish_command(self, steering: int, throttle: int):
        cmd = f"{steering},{throttle}"
        m = String()
        m.data = cmd
        self.pub_cmd.publish(m)
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

