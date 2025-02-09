#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from math import sqrt, atan
from std_msgs.msg import Float32
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point

from utils.lateral_controller import Pure_Pursuit
from utils.longitudinal_controller import PI_Speed_controller

# GPIO 핀 설정 (MD20A, MD10C)
PWM_PIN = 18  # 모터 PWM 출력 핀
DIR_PIN = 23  # 모터 방향 제어 핀
SERVO_PIN = 13  # 서보모터(조향) 핀
PWM_FREQ = 1000  # PWM 주파수 (모터)

class Controller():
    def __init__(self):
        rospy.init_node("controller")

        # GPIO 설정
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PWM_PIN, GPIO.OUT)
        GPIO.setup(DIR_PIN, GPIO.OUT)
        GPIO.setup(SERVO_PIN, GPIO.OUT)

        # PWM 초기화
        try:
            self.pwm_motor = GPIO.PWM(PWM_PIN, PWM_FREQ)
            self.pwm_initialized = True
        except RuntimeError as e:
            rospy.logerr(f"GPIO PWM Initialization Failed: {e}")
            self.pwm_initialized = False
        if not self.pwm_initialized:
            retry_attempts = 3
        for attempt in range(retry_attempts):
            try:
                self.pwm_motor = GPIO.PWM(PWM_PIN, PWM_FREQ)
                self.pwm_initialized = True
                break
            except RuntimeError as e:
                rospy.logerr(f"Attempt {attempt+1}: PWM initialization failed: {e}")
                if attempt == retry_attempts - 1:
                    rospy.logerr("All PWM initialization attempts failed. Controller will not execute motor commands.")
                    self.pwm_initialized = False
                    return
            return
            return
        except RuntimeError as e:
            rospy.logerr(f"GPIO PWM Initialization Failed: {e}")
            return
        self.pwm_motor.start(0)  # 초기 속도 0%

        self.pwm_servo = GPIO.PWM(SERVO_PIN, 50)  # 서보 모터는 50Hz
        self.pwm_servo.start(7.5)  # 중앙 위치 (7.5% 듀티 사이클)

        # ROS 구독자 설정
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber('odom', Odometry, self.odom_callback)

        # 속도 및 조향 값
        self.desired_speed = 0
        if self.current_speed > 0:
            if self.current_speed > 0:
            rospy.logwarn("Gradually reducing speed instead of abrupt reset.")
            while self.desired_speed > 0:
                self.desired_speed -= 0.1
                rospy.sleep(0.1)  # Reduce speed gradually
        self.current_speed = 0
        self.desired_speed = 0  # Ensure desired speed is reset as well
        self.steering_angle = 0
        self.lfd = 2.2 * sqrt(1e-1)

        # 컨트롤러 초기화
        self.speed_controller = PI_Speed_controller()
        self.lateral_controller_pure_pursuit = Pure_Pursuit()

        rate = rospy.Rate(40)  # 40Hz

        while not rospy.is_shutdown():
            self.lateral_control()
            self.longitudinal_control()
            self.publish_commands()
            rate.sleep()

    def longitudinal_control(self):
        if self.current_speed is None:
            self.current_speed = 0
            return

        dt = 0.025  # 40Hz 기준 (1/40)
        self.longitudinal_command = self.speed_controller.command(self.desired_speed, self.current_speed, dt)

        # 방향 및 PWM 설정
        if self.longitudinal_command >= 0:
            GPIO.output(DIR_PIN, GPIO.HIGH)  # 정방향
        else:
            GPIO.output(DIR_PIN, GPIO.LOW)  # 역방향

        duty_cycle = min(abs(self.longitudinal_command) * 100, 100)
        self.pwm_motor.ChangeDutyCycle(duty_cycle)

    def lateral_control(self):
        if self.current_speed is None:
            return

        # 속도 기반 Look-ahead Distance(LFD) 계산
        if not rospy.has_param("/lfd_factor"):
            if not rospy.has_param("/lfd_factor"):
            rospy.set_param("/lfd_factor", 2.2)
            rospy.logwarn("/lfd_factor not set. Setting default to 2.2")
            rospy.logwarn("/lfd_factor not set. Setting default to 2.2")
            rospy.logwarn("/lfd_factor not set. Using default value 2.2")
        self.lfd_factor = rospy.get_param("/lfd_factor", 2.2)
        self.lfd = self.lfd_factor * sqrt(max(self.current_speed, 1e-1))

        for waypoint in self.local_path.poses:
            path_point = waypoint.pose.position
            dis = sqrt(pow(path_point.x, 2) + pow(path_point.y, 2))

            if dis >= self.lfd:
                self.look_ahead_point = path_point
                self.steering_angle = self.lateral_controller_pure_pursuit.command(self.look_ahead_point)
                break

        # 서보 모터 조향값 변환
        servo_pwm = 7.5 + (self.steering_angle * 2.5)
        servo_min = rospy.get_param("/servo_min", 5.0)
        servo_max = rospy.get_param("/servo_max", 10.0)
        if not rospy.has_param("/servo_min") or not rospy.has_param("/servo_max"):
            if not rospy.has_param("/servo_min"):
            rospy.set_param("/servo_min", 5.0)
            rospy.logwarn("/servo_min not set. Setting default to 5.0")
        if not rospy.has_param("/servo_max"):
            rospy.set_param("/servo_max", 10.0)
            rospy.logwarn("/servo_max not set. Setting default to 10.0")
            rospy.set_param("/servo_max", 10.0)
            rospy.logwarn("Servo PWM range not set. Using defaults: 5.0 - 10.0")
        servo_pwm = max(rospy.get_param("/servo_min"), min(servo_pwm, rospy.get_param("/servo_max")))
        self.pwm_servo.ChangeDutyCycle(servo_pwm)

    def publish_commands(self):
        rospy.loginfo(f"Speed: {self.longitudinal_command}, Steering: {self.steering_angle}")

    def path_callback(self, msg):
        self.local_path = msg

    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x  # 차량 속도 (m/s)

    def cleanup(self):
        self.pwm_motor.stop()
        self.pwm_servo.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    try:
        controller = Controller()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.cleanup()
