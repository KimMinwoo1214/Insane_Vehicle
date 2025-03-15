#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import math
import csv
import os
import utm  # Turbo87의 utm 모듈

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # CSV 파일 경로 (현재 파일과 같은 디렉토리에 위치한다고 가정)
        csv_file_path = os.path.join(os.path.dirname(__file__), "waypoints.csv")
        self.waypoints = self.load_waypoints(csv_file_path)

        if not self.waypoints:
            self.get_logger().error("Waypoint 데이터를 로드할 수 없습니다.")
            rclpy.shutdown()
            return

        self.current_waypoint_index = 0

        # GPS 데이터 수신 구독자 (sensor_msgs/NavSatFix)
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        # 제어 명령(String)을 발행하는 퍼블리셔 (teleop_commands 토픽)
        self.cmd_pub = self.create_publisher(String, 'teleop_commands', 10)

        # 목표 waypoint에 도달할 때까지의 lookahead 거리 (미터 단위, 필요시 튜닝)
        self.lookahead_distance = 1.0
        # 기본 throttle 값 (0~30 범위; 필요에 따라 조정)
        self.base_throttle = 20

        # 기준점 설정: 첫 번째 waypoint를 기준점으로 사용하고, UTM 좌표를 계산
        self.ref_lat = self.waypoints[0][0]
        self.ref_lon = self.waypoints[0][1]
        self.ref_easting, self.ref_northing, self.ref_zone_number, self.ref_zone_letter = utm.from_latlon(self.ref_lat, self.ref_lon)
        self.get_logger().info(f"기준점: {self.ref_lat}, {self.ref_lon} -> UTM: {self.ref_easting:.2f}, {self.ref_northing:.2f} Zone {self.ref_zone_number}{self.ref_zone_letter}")

    def load_waypoints(self, filename):
        """
        CSV 파일에서 waypoint를 파싱합니다.
        CSV 파일은 헤더에 INDEX, Lat, Long 컬럼을 포함한다고 가정합니다.
        """
        waypoints = []
        try:
            with open(filename, mode='r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    lat = float(row["Lat"])
                    lon = float(row["Long"])
                    waypoints.append((lat, lon))
            self.get_logger().info(f"로드된 waypoint 수: {len(waypoints)}")
            return waypoints
        except Exception as e:
            self.get_logger().error(f"CSV 파일에서 waypoint 로드 중 오류 발생: {e}")
            return []

    def gps_to_xy(self, lat, lon):
        """
        utm 모듈을 사용해 위도/경도를 UTM 좌표 (easting, northing)로 변환한 후,
        기준점(self.ref_easting, self.ref_northing)으로부터의 상대 좌표(x, y)를 계산합니다.
        """
        easting, northing, zone_number, zone_letter = utm.from_latlon(lat, lon)
        # 보통 동일 지역(예: Zone 52N) 내에 있음을 가정합니다.
        x = easting - self.ref_easting
        y = northing - self.ref_northing
        return x, y

    def gps_callback(self, msg):
        # 현재 GPS 좌표를 로컬 좌표로 변환
        current_x, current_y = self.gps_to_xy(msg.latitude, msg.longitude)

        # 모든 waypoint 도달 여부 확인
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("모든 waypoint 도달. 정지합니다.")
            self.publish_cmd(90, 0)  # 스티어링 중앙(90), throttle 0
            return

        # 현재 목표 waypoint 선택 및 좌표 변환
        target_lat, target_lon = self.waypoints[self.current_waypoint_index]
        target_x, target_y = self.gps_to_xy(target_lat, target_lon)
        # 현재 위치와 목표 waypoint 사이의 거리 계산
        distance = math.hypot(target_x - current_x, target_y - current_y)

        # lookahead 거리 내로 들어오면 다음 waypoint로 전환
        if distance < self.lookahead_distance:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index+1} 도달")
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info("최종 waypoint 도달. 정지합니다.")
                self.publish_cmd(90, 0)
                return
            target_lat, target_lon = self.waypoints[self.current_waypoint_index]
            target_x, target_y = self.gps_to_xy(target_lat, target_lon)
            distance = math.hypot(target_x - current_x, target_y - current_y)

        # 목표 waypoint까지의 방향(heading) 계산 (현재 차량의 heading 정보는 없으므로 단순화)
        path_angle = math.atan2(target_y - current_y, target_x - current_x)
        heading_error = path_angle

        # P-controller로 angular_z 산출
        Kp = 1.0  # 필요에 따라 튜닝
        angular_z = Kp * heading_error

        # 계산된 angular_z를 기반으로 스티어링 명령 생성  
        # 양수(좌회전): multiplier 33, 음수(우회전): multiplier 36
        if angular_z >= 0:
            steering = 90 + int(angular_z * 33)
        else:
            steering = 90 + int(angular_z * 36)
        # 스티어링 범위를 [54, 123]으로 제한
        steering = max(54, min(123, steering))

        # throttle은 고정 값 (목표 주행 속도)
        throttle = self.base_throttle

        self.publish_cmd(steering, throttle)

        self.get_logger().info(
            f"Waypoint {self.current_waypoint_index+1} 목표, 거리: {distance:.2f} m, steering: {steering}, throttle: {throttle}"
        )

    def publish_cmd(self, steering, throttle):
        msg = String()
        msg.data = f"{steering},{throttle}"
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("노드 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

