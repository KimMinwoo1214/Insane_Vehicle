import numpy as np
import cv2  # 칼만 필터를 위해 OpenCV 사용


def calculate_cluster_centers(labels, points):
    """
    클러스터 ID별로 중심을 계산하는 함수.

    Args:
        labels (np.ndarray): 클러스터 라벨 배열, 각 점의 ID를 포함 (-1은 노이즈).
        points (np.ndarray): 각 점의 (x, y) 좌표 배열.

    Returns:
        dict: {클러스터 ID: 중심 좌표(x, y)} 형태의 딕셔너리.
    """
    cluster_centers = {}
    unique_labels = np.unique(labels)  # 클러스터 ID 가져오기

    for label in unique_labels:
        if label == -1:
            continue  # 노이즈는 무시
        cluster_points = points[labels == label]  # 해당 클러스터의 점들 추출
        center = np.mean(cluster_points, axis=0)  # 중심 좌표 계산
        cluster_centers[label] = tuple(center)

    return cluster_centers

class ClusterTracker:
    def __init__(self):
        """
        클러스터 및 추적 정보를 관리하는 클래스.
        """
        self.cluster_centers = {}  # {클러스터 ID: 중심 좌표}
        self.tracked_center = None  # 추적 중인 클러스터 중심 좌표
        self.kalman_filter = None  # 칼만 필터 객체
        self.is_tracking = False  # 추적 상태 초기화
        self.latest_labels = None  # 최신 클러스터 라벨
        self.latest_points = None  # 최신 클러스터 점들

    def initialize_kalman_filter(self, initial_position):
        """
        칼만 필터를 초기화하고 추적을 활성화합니다.

        Args:
            initial_position (tuple): 초기 중심 좌표 (x, y).
        """
        self.kalman_filter = cv2.KalmanFilter(4, 2)  # 상태(x, y, vx, vy), 관측값(x, y)
        self.kalman_filter.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        self.kalman_filter.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        self.kalman_filter.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
        self.kalman_filter.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
        self.kalman_filter.statePost = np.array([initial_position[0], initial_position[1], 0, 0], dtype=np.float32)

        self.tracked_center = initial_position  # 추적할 중심 좌표 저장
        self.is_tracking = True  # 추적 활성화

    def track(self):
        """
        칼만 필터를 이용하여 추적.

        Returns:
            np.ndarray: 추적된 클러스터의 예측 좌표 (x, y).
        """
        if not self.is_tracking or self.kalman_filter is None:
            return None

        # 상태 예측
        predicted_state = self.kalman_filter.predict()
        return predicted_state[:2]

    def update_clusters(self, labels, points):
        """
        클러스터 중심을 업데이트하고 추적 중인 중심을 업데이트합니다.

        Args:
            labels (np.ndarray): 클러스터 라벨 배열.
            points (np.ndarray): 각 점의 (x, y) 좌표 배열.
        """
        self.cluster_centers = calculate_cluster_centers(labels, points)
        self.latest_labels = labels
        self.latest_points = points

        if self.is_tracking and self.tracked_center is not None:
            # 추적 중인 중심과 클러스터 중심 비교
            distances = {
                cluster_id: np.linalg.norm(np.array(center) - np.array(self.tracked_center))
                for cluster_id, center in self.cluster_centers.items()
            }

            # 가장 가까운 중심 선택 (추적 유지)
            closest_id, closest_distance = min(distances.items(), key=lambda x: x[1])
            if closest_distance < 50:  # 거리 임계값
                self.tracked_center = self.cluster_centers[closest_id]
                self.kalman_filter.correct(np.array(self.tracked_center, dtype=np.float32))

    def select_tracked_center(self, click_x, click_y, selection_radius=50):
        """
        클릭된 위치와 가장 가까운 클러스터 중심을 추적 대상으로 설정.

        Args:
            click_x (float): 클릭한 x 좌표.
            click_y (float): 클릭한 y 좌표.
            selection_radius (float): 선택 가능한 반경.

        Returns:
            tuple: 선택된 중심 좌표 (x, y), 없으면 None.
        """
        min_distance = float('inf')
        selected_center = None

        for center in self.cluster_centers.values():
            distance = np.linalg.norm(np.array(center) - np.array([click_x, click_y]))
            if distance < min_distance and distance <= selection_radius:
                min_distance = distance
                selected_center = center

        if selected_center is not None:
            self.initialize_kalman_filter(selected_center)

        return selected_center

    def get_tracked_cluster_points(self):
        """
        현재 추적 중인 클러스터에 속한 모든 점을 반환합니다.

        Returns:
            np.ndarray: 추적 중인 클러스터에 속한 점들의 배열, 없으면 None.
        """
        if self.tracked_center is None or self.latest_labels is None or self.latest_points is None:
            return None

        # 추적 중인 중심에 가장 가까운 클러스터의 라벨 찾기
        distances = {
            cluster_id: np.linalg.norm(np.array(center) - np.array(self.tracked_center))
            for cluster_id, center in self.cluster_centers.items()
        }
        closest_id = min(distances, key=distances.get)

        # 해당 클러스터의 모든 점 반환
        cluster_indices = self.latest_labels == closest_id
        return self.latest_points[cluster_indices]



    def get_tracked_center(self):
        """
        추적 중인 클러스터의 중심 좌표를 반환.

        Returns:
            tuple: 추적 중인 클러스터 중심 좌표 (x, y), 없으면 None.
        """
        if self.tracked_id is not None and self.tracked_id in self.cluster_centers:
            return self.cluster_centers[self.tracked_id]
        return None