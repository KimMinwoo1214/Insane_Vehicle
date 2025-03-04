import numpy as np
import cv2
# from hdbscan import HDBSCAN
import time
import csv
import os
from datetime import datetime
from sklearn.cluster import DBSCAN # , OPTICS


def polar_to_cartesian(scan_data):
    """
    LaserScan 메시지 데이터를 데카르트 좌표계로 변환하고, 90도 왼쪽으로 회전.

    Args:
        scan_data (LaserScan): ROS2 LaserScan 메시지.

    Returns:
        tuple:
            - np.ndarray: 변환된 (x, y) 좌표 배열, 크기 (N, 2)
            - np.ndarray: 변환된 점들의 실제 거리 (ranges)
    """
    num_ranges = len(scan_data.ranges)
    angles = np.arange(scan_data.angle_min, scan_data.angle_max, scan_data.angle_increment)
    angles = angles[:num_ranges]  # ranges 길이에 맞춰 각도 배열 조정

    ranges = np.array(scan_data.ranges)

    # 유효하지 않은 거리 값 필터링 (range_min과 range_max 기준)
    valid_indices = np.isfinite(ranges) & (ranges >= scan_data.range_min) & (ranges <= scan_data.range_max)
    ranges = ranges[valid_indices]
    angles = angles[valid_indices]

    # 극좌표 → 데카르트 좌표 변환
    x_coords = ranges * np.cos(angles)
    y_coords = ranges * np.sin(angles)

    points = np.stack((x_coords, y_coords), axis=1)  # (N, 2) 형태

    # 90도 왼쪽 회전 변환 적용
    rotation_matrix = np.array([[0, -1], [1, 0]])  # 90도 회전 행렬
    rotated_points = points @ rotation_matrix.T  # 행렬 곱으로 회전 적용

    return rotated_points, ranges



# from sklearn.cluster import MeanShift

# def mean_shift_clustering(scan_data, bandwidth=None):
#     points = polar_to_cartesian(scan_data)
#     start_time = time.time()
#     mean_shift = MeanShift(bandwidth=bandwidth)
#     labels = mean_shift.fit_predict(points)
#     execution_time = time.time() - start_time
#     return labels, points, execution_time

# from sklearn.cluster import AffinityPropagation

# def affinity_propagation_clustering(scan_data, damping=0.9, preference=None):
#     points = polar_to_cartesian(scan_data)
#     start_time = time.time()
#     affinity_propagation = AffinityPropagation(damping=damping, preference=preference)
#     labels = affinity_propagation.fit_predict(points)
#     execution_time = time.time() - start_time
#     return labels, points, execution_time

# def hdbscan_clustering(scan_data, min_samples=5, min_cluster_size=5):
#     """
#     HDBSCAN을 사용하여 스캔 데이터를 클러스터링.

#     Args:
#         scan_data (LaserScan): ROS2 LaserScan 메시지.
#         min_samples (int): HDBSCAN의 최소 샘플 개수 (default: 5).
#         min_cluster_size (int): 최소 클러스터 크기 (default: 5).

#     Returns:
#         labels (np.ndarray): 유효 데이터의 클러스터 레이블 (-1은 노이즈).
#         cluster_points (np.ndarray): 유효 데이터의 클러스터링된 좌표.
#         execution_time (float): 클러스터링 수행 시간.
#     """
#     points = polar_to_cartesian(scan_data)  # LaserScan 데이터를 데카르트 좌표로 변환

#     # HDBSCAN 클러스터링 실행 및 시간 측정
#     start_time = time.time()
#     hdbscan = HDBSCAN(min_samples=min_samples, min_cluster_size=min_cluster_size)
#     labels = hdbscan.fit_predict(points)
#     execution_time = time.time() - start_time

#     return labels, points, execution_time

# def optics_clustering(scan_data, min_samples=5, max_eps=np.inf, cluster_method='xi', xi=0.1):
#     """
#     OPTICS를 사용하여 스캔 데이터를 클러스터링.

#     Args:
#         scan_data (LaserScan): ROS2 LaserScan 메시지.
#         min_samples (int): OPTICS의 최소 샘플 개수.
#         max_eps (float): OPTICS의 최대 반경 거리 (default: inf).
#         cluster_method (str): 클러스터링 방법 ('xi' 또는 'dbscan', default: 'xi').
#         xi (float): 클러스터링의 밀도 변화율 (default: 0.05).

#     Returns:
#         labels (np.ndarray): 유효 데이터의 클러스터 레이블 (-1은 노이즈).
#         cluster_points (np.ndarray): 유효 데이터의 클러스터링된 좌표.
#         execution_time (float): 클러스터링 수행 시간.
#     """
#     points = polar_to_cartesian(scan_data)  # LaserScan 데이터를 데카르트 좌표로 변환

#     # OPTICS 클러스터링 실행 및 시간 측정
#     start_time = time.time()
#     optics = OPTICS(min_samples=min_samples, max_eps=max_eps, cluster_method=cluster_method, xi=xi)
#     optics.fit(points)
#     execution_time = time.time() - start_time
#     labels = optics.labels_

#     return labels, points, execution_time


def dbscan_clustering(scan_data, epsilon=12, min_samples=5):
    """
    DBSCAN을 사용하여 스캔 데이터를 클러스터링.

    Args:
        scan_data (LaserScan): ROS2 LaserScan 메시지.
        epsilon (float): DBSCAN의 epsilon (근접 거리 허용 범위).
        min_samples (int): DBSCAN의 최소 샘플 개수.

    Returns:
        labels (np.ndarray): 유효 데이터의 클러스터 레이블 (-1은 노이즈).
        cluster_points (np.ndarray): 유효 데이터의 클러스터링된 좌표.
        execution_time (float): 클러스터링 수행 시간.
    """
    points, distances = polar_to_cartesian(scan_data)  # LaserScan 데이터를 데카르트 좌표로 변환

    # DBSCAN 클러스터링 실행 및 시간 측정
    start_time = time.time()
    dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
    labels = dbscan.fit_predict(points)
    execution_time = time.time() - start_time

    return labels, points, distances, execution_time


def record_execution_time(algorithm_name, execution_time, execution_times):
    """
    클러스터링 실행 시간을 기록.

    Args:
        algorithm_name (str): 클러스터링 알고리즘 이름 ("DBSCAN" or "OPTICS").
        execution_time (float): 클러스터링 수행 시간.
        execution_times (list): 실행 시간 기록 리스트.

    Returns:
        None
    """
    execution_times.append(execution_time)
    print(f"[{algorithm_name}] Execution Time: {execution_time:.4f} seconds")


def save_execution_statistics(algorithm_name, execution_times):
    """
    실행 시간 통계를 CSV 파일로 저장.

    Args:
        algorithm_name (str): 클러스터링 알고리즘 이름 ("DBSCAN" or "OPTICS").
        execution_times (list): 실행 시간 기록 리스트.

    Returns:
        None
    """
    if not execution_times:
        print(f"No execution times recorded for {algorithm_name}.")
        return

    # 통계 계산
    avg_time = sum(execution_times) / len(execution_times)
    min_time = min(execution_times)
    max_time = max(execution_times)

    # CSV 파일에 저장
    file_name = f"{algorithm_name}_execution_times.csv"
    file_path = os.path.join(os.getcwd(), file_name)

    with open(file_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Execution Time (s)"])
        for exec_time in execution_times:
            writer.writerow([datetime.now().strftime("%Y-%m-%d %H:%M:%S"), exec_time])

        # 통계 추가
        writer.writerow([])
        writer.writerow(["Statistics"])
        writer.writerow(["Average Time", avg_time])
        writer.writerow(["Minimum Time", min_time])
        writer.writerow(["Maximum Time", max_time])

    print(f"Execution statistics saved to {file_path}")


def world_to_pixel(labels, world_points, max_distance, canvas_size=800, padding=50):
    """
    월드 좌표계 점과 라벨을 픽셀 좌표계로 변환하는 함수.

    Args:
        labels (np.ndarray): 클러스터 라벨 배열.
        world_points (np.ndarray): 월드 좌표계 점들의 배열, [[x, y], ...].
        max_distance (float): 캔버스에서 표현 가능한 최대 거리.
        canvas_size (int, optional): 캔버스의 크기 (정사각형 기준). 기본값은 800.
        padding (int, optional): 캔버스 여백. 기본값은 50.

    Returns:
        tuple: (픽셀 라벨 배열, 픽셀 좌표 배열)
    """
    if len(world_points) == 0:
        return labels, np.empty((0, 2))

    # 캔버스 중심 계산
    canvas_center = np.array([canvas_size / 2, canvas_size / 2])

    # 월드 좌표 -> 픽셀 좌표 스케일링
    scale = (canvas_size - 2 * padding) / (2 * max_distance)  # 월드 -> 캔버스 스케일 계산
    pixel_points = world_points * scale  # 월드 좌표를 캔버스 크기로 변환
    pixel_points[:, 1] *= -1  # Y축 반전 (화면 좌표계는 아래로 증가, 월드 좌표계는 위로 증가)
    pixel_points += canvas_center  # 캔버스 중심으로 이동

    # 라벨은 변하지 않으므로 그대로 반환
    return labels, pixel_points




# def display_clusters(labels, cluster_points, max_distance=5, base_canvas_size=800, padding=50, color_vis=False):
#     """
#     클러스터링된 점들을 검은 바탕 이미지에 그린 후 반환하며,
#     색상 시각화를 조정할 수 있는 매개변수 추가 (color_vis).

#     Args:
#         labels (np.ndarray): 클러스터 라벨.
#         cluster_points (np.ndarray): 클러스터링된 점들의 좌표.
#         max_distance (float): 캔버스의 최대 거리.
#         base_canvas_size (int): 기본 캔버스 크기.
#         padding (int): 캔버스 여백.
#         color_vis (bool): True면 클러스터별 색상, False면 전부 흰색.

#     Returns:
#         np.ndarray: 시각화를 위한 캔버스 이미지.
#     """
#     canvas_size = base_canvas_size
#     canvas = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)

#     if len(cluster_points) > 0:
#         # 캔버스에 데이터 비율 맞추기
#         scale = (canvas_size - 2 * padding) / (2 * max_distance)
#         canvas_center = np.array([canvas_size / 2, canvas_size / 2])

#         normalized_points = cluster_points * scale
#         normalized_points[:, 1] *= -1
#         normalized_points += canvas_center

#         # 클러스터별 색상 생성
#         unique_labels = np.unique(labels)
#         colors = generate_colors(len(unique_labels))

#         for label in unique_labels:
#             label_indices = (labels == label)
#             cluster_points_label = normalized_points[label_indices]

#             # 색상 설정: color_vis가 False일 경우 흰색
#             color = (255, 255, 255) if not color_vis else (128, 128, 128) if label == -1 else colors[label]

#             for point in cluster_points_label:
#                 x, y = int(point[0]), int(point[1])
#                 if 0 <= x < canvas_size and 0 <= y < canvas_size:
#                     cv2.circle(canvas, (x, y), 2, color, -1)

#             if color_vis and label != -1:
#                 cluster_center = np.mean(cluster_points_label, axis=0)
#                 center_x, center_y = int(cluster_center[0]), int(cluster_center[1])
#                 cv2.circle(canvas, (center_x, center_y), 4, (0, 255, 0), -1)
#                 cv2.putText(
#                     canvas,
#                     str(label),
#                     (center_x, center_y - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     0.7,
#                     (0, 255, 0),
#                     2,
#                     cv2.LINE_AA,
#                 )

#     return canvas

# def display_clusters(
#     labels, cluster_points, max_distance=5, canvas_size=800, padding=50, color_vis=True, only_tracking=False, tracked_id=None
# ):
#     """
#     클러스터링된 점들을 시각화하는 함수.

#     Args:
#         labels (np.ndarray): 클러스터 라벨.
#         cluster_points (np.ndarray): 클러스터링된 점들의 좌표.
#         max_distance (float): 캔버스의 최대 거리.
#         base_canvas_size (int): 기본 캔버스 크기.
#         padding (int): 캔버스 여백.
#         color_vis (bool): True면 클러스터별 색상, False면 전부 흰색.
#         only_tracking (bool): True면 추적 중인 클러스터를 강조.
#         tracked_id (int): 추적 중인 클러스터 ID (없을 경우 None).

#     Returns:
#         np.ndarray: 시각화를 위한 캔버스 이미지.
#     """
#     canvas = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)

#     if len(cluster_points) > 0:
#         # 캔버스에 데이터 비율 맞추기
#         scale = (canvas_size - 2 * padding) / (2 * max_distance)
#         canvas_center = np.array([canvas_size / 2, canvas_size / 2])

#         normalized_points = cluster_points * scale
#         normalized_points[:, 1] *= -1
#         normalized_points += canvas_center

#         # 클러스터별 색상 생성
#         unique_labels = np.unique(labels)
#         colors = generate_colors(len(unique_labels))

#         for label in unique_labels:
#             # 해당 클러스터의 점들 가져오기
#             label_indices = (labels == label)
#             cluster_points_label = normalized_points[label_indices]

#             # 색상 설정
#             if only_tracking:
#                 if label == tracked_id:  # 추적 중인 클러스터
#                     point_color = (255, 0, 0)  # 파란색 (추적 중인 점)
#                     center_color = (255, 0, 0)  # 파란색 (중심 원)
#                     id_color = (0, 0, 255)  # 빨간색 (ID)
#                 else:  # 다른 클러스터
#                     point_color = (255, 255, 255) if label != -1 else  (128, 128, 128) 
#                     center_color = (0, 255, 0)  
#                     id_color = (0, 255, 0)  # 초록색 (ID)
#             else:  # 일반 시각화
#                 point_color = (255, 255, 255) if not color_vis else (128, 128, 128) if label == -1 else colors[label]
#                 center_color = (0, 255, 0) if label != -1 else None
#                 id_color = (0, 255, 0)

#             # 클러스터 점들 그리기
#             for point in cluster_points_label:
#                 x, y = int(point[0]), int(point[1])
#                 if 0 <= x < canvas_size and 0 <= y < canvas_size:
#                     cv2.circle(canvas, (x, y), 2, point_color, -1)

#             # 중심점 및 클러스터 ID 표시
#             if label != -1:  # 노이즈 제외
#                 cluster_center = np.mean(cluster_points_label, axis=0)
#                 center_x, center_y = int(cluster_center[0]), int(cluster_center[1])

#                 if center_color:  # 중심 원
#                     cv2.circle(canvas, (center_x, center_y), 5, center_color, -1)

#                 # 클러스터 ID 텍스트
#                 cv2.putText(
#                     canvas,
#                     str(label),
#                     (center_x, center_y - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     0.7,
#                     id_color,
#                     2,
#                     cv2.LINE_AA,
#                 )

#     return canvas


def display_clusters(
    labels, cluster_points, canvas_size=800, cluster_tracker=None, only_clustering=False
):
    """
    클러스터링된 점들을 시각화하는 함수.

    Args:
        labels (np.ndarray): 클러스터 라벨.
        cluster_points (np.ndarray): 픽셀 좌표계 점들의 배열.
        canvas_size (int, optional): 캔버스의 크기 (정사각형 기준). 기본값은 800.
        cluster_tracker (ClusterTracker, optional): 클러스터 추적 객체.
        only_clustering (bool, optional): True면 클러스터링에 따라 색상을 설정하고 추적 강조를 비활성화.

    Returns:
        np.ndarray: 시각화를 위한 캔버스 이미지.
    """
    canvas = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)

    if len(cluster_points) > 0:
        unique_labels = np.unique(labels)

        # 클러스터 색상 생성
        if only_clustering:
            colors = generate_colors(len(unique_labels))  # 클러스터링별 색상

        # 추적 중인 클러스터 점 가져오기
        tracked_cluster_points = (
            cluster_tracker.get_tracked_cluster_points()
            if cluster_tracker and cluster_tracker.is_tracking
            else None
        )

        for label in unique_labels:
            label_indices = (labels == label)
            cluster_points_label = cluster_points[label_indices]

            # 클러스터 중심 계산
            cluster_center = np.mean(cluster_points_label, axis=0)
            center_x, center_y = int(cluster_center[0]), int(cluster_center[1])

            if only_clustering:  # 클러스터링 모드
                if label == -1:  # 이상치
                    point_color = (128, 128, 128)  # 회색
                    center_color = None
                    id_color = None
                else:
                    point_color = colors[label]  # 클러스터별 색상
                    center_color = (0, 255, 0)  # 초록색
                    id_color = (0, 255, 0)  # 초록색
            else:  # 추적 강조 모드
                # 현재 클러스터가 추적 중인지 확인
                is_tracked_cluster = (
                    tracked_cluster_points is not None and
                    np.all(np.isin(cluster_points_label, tracked_cluster_points))
                )

                # 색상 설정
                if label == -1:  # 이상치
                    point_color = (128, 128, 128)  # 회색
                    center_color = None
                    id_color = None
                elif is_tracked_cluster:  # 추적 중인 클러스터
                    point_color = (255, 0, 0)  # 파란색
                    center_color = (0, 0, 255)  # 빨간색
                    id_color = (0, 0, 255)  # 빨간색
                else:  # 다른 클러스터
                    point_color = (255, 255, 255)  # 흰색
                    center_color = (0, 255, 0)  # 초록색
                    id_color = (0, 255, 0)  # 초록색

            # 클러스터 점 그리기
            for point in cluster_points_label:
                x, y = int(point[0]), int(point[1])
                if 0 <= x < canvas_size and 0 <= y < canvas_size:
                    cv2.circle(canvas, (x, y), 2, point_color, -1)

            # 중심점 표시
            if center_color:
                cv2.circle(canvas, (center_x, center_y), 5, center_color, -1)

            # 클러스터 ID 표시
            if id_color:
                cv2.putText(
                    canvas,
                    str(label),
                    (center_x, center_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    id_color,
                    2,
                    cv2.LINE_AA,
                )

    return canvas





def generate_colors(num_colors):
    """
    고유 라벨의 개수만큼 색상을 생성합니다.

    Args:
        num_colors (int): 생성할 색상의 개수.

    Returns:
        dict: {라벨: (B, G, R)} 형식의 색상 맵.
    """
    np.random.seed(0)
    colors = np.random.randint(0, 255, size=(num_colors, 3), dtype=np.uint8)
    return {i: tuple(map(int, colors[i])) for i in range(num_colors)}

