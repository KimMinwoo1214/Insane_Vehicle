import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
from filterpy.kalman import ExtendedKalmanFilter as EKF

try:
    import pyapriltags as apriltag
except ImportError:
    raise ImportError("pyapriltags 패키지가 설치되지 않았습니다. `pip install pyapriltags` 실행 후 다시 시도하세요.")

from CalibJ.utils.config_loader import save_extrinsic_to_json, load_json
from CalibJ.module.calibration_module import calibration_2dlidar_camera

class LiDARCameraCalibration(Node):
    def __init__(self):
        super().__init__('lidar_camera_calibration')

        self.detector = apriltag.Detector(families='tag36h11')

        self.camera_matrix = np.array([
            [703.37906585,   0.,         330.37487405],
            [  0.,         750.72854219, 226.5012125 ],
            [  0.,           0.,           1.        ]
        ])
        
        self.dist_coeffs = np.array([[ 8.65114817e-02,  5.75780539e-01, -3.92050613e-03,  
                                       2.34661487e-03, -2.74255703e+00]])

        # ✅ 확장 칼만 필터(EKF) 초기화
        self.ekf = EKF(dim_x=4, dim_z=2)  # 4D 상태 (x, y, vx, vy), 2D 측정 (x, y)
        self.ekf.x = np.zeros(4)  # 초기 상태 벡터
        self.ekf.F = np.array([[1, 0, 1, 0],  # 상태 전이 행렬
                               [0, 1, 0, 1],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]])
        self.ekf.H = np.array([[1, 0, 0, 0],  # 측정 행렬
                               [0, 1, 0, 0]])
        self.ekf.P *= 1000  # 초기 공분산 설정
        self.ekf.R = np.eye(2) * 0.1  # 측정 잡음 공분산

        self.lidar_points = None

        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.capture = cv2.VideoCapture(2)
        if not self.capture.isOpened():
            self.get_logger().error("카메라 초기화 실패!")
            raise RuntimeError("Camera initialization failed")

        self.get_logger().info("AprilTag 기반 캘리브레이션 준비 완료!")

    def HJacobian(self, x):
        """확장 칼만 필터의 Jacobian 행렬 반환"""
        return np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0]])

    def Hx(self, x):
        """예측된 상태 벡터를 측정 공간으로 변환"""
        return np.array([x[0], x[1]])

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        self.lidar_points = np.column_stack((x[valid], y[valid]))

    def detect_apriltag_and_lidar(self):
        ret, frame = self.capture.read()
        if not ret:
            self.get_logger().error("카메라 프레임 캡처 실패!")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        apriltag_detections = self.detector.detect(gray)

        if len(apriltag_detections) == 0:
            self.get_logger().error("AprilTag 검출 실패!")
            return

        self.get_logger().info(f"AprilTag {len(apriltag_detections)}개 검출 완료")

        if self.lidar_points is None or len(self.lidar_points) == 0:
            self.get_logger().error("LiDAR 데이터가 없습니다!")
            return

        db = DBSCAN(eps=0.1, min_samples=3).fit(self.lidar_points)
        labels = db.labels_
        filtered_points = self.lidar_points[labels != -1]

        self.ekf.predict()

        for i, (x, y) in enumerate(filtered_points):
            self.ekf.update([x, y], self.HJacobian, self.Hx)  # ✅ 수정된 업데이트 방식

        self.lidar_features = filtered_points
        self.apriltag_features = apriltag_detections

    def perform_calibration(self):
        if self.lidar_features.size == 0 or len(self.apriltag_features) == 0:
            self.get_logger().error("캘리브레이션을 수행할 데이터가 부족합니다!")
            return
        apriltag_centers = np.array([tag.center for tag in self.apriltag_features], dtype=np.float32)  # ⬅ centers만 추출
    	
        success, rvec, tvec = calibration_2dlidar_camera(
            self.lidar_features, 
            apriltag_centers, 
            self.camera_matrix, 
            self.dist_coeffs
        )

        if success:
            self.rvec = rvec
            self.tvec = tvec
            self.get_logger().info("캘리브레이션 성공! 변환 행렬 저장 중...")
            save_extrinsic_to_json("calibration_extrinsic.json", rvec, tvec)
            self.get_logger().info("변환 행렬 저장 완료!")
        else:
            self.get_logger().error("캘리브레이션 실패!")

    def load_calibration(self):
        if os.path.exists("calibration_extrinsic.json"):
            extrinsic_data = load_json("calibration_extrinsic.json")
            self.rvec = extrinsic_data["rvec"]
            self.tvec = extrinsic_data["tvec"]
            self.get_logger().info("캘리브레이션 결과 불러오기 성공!")
        else:
            self.get_logger().error("calibration_extrinsic.json 파일이 존재하지 않습니다!")

def main():
    rclpy.init()
    node = LiDARCameraCalibration()
    
    while rclpy.ok():
        rclpy.spin_once(node)
        node.detect_apriltag_and_lidar()
        node.perform_calibration()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

