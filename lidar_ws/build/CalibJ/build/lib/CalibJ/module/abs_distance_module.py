import matplotlib.pyplot as plt
import numpy as np
def show_pixel_spectrum(self, projected_points, frame_width, frame_height):
    """
    Show the spectrum of pixel coordinates for LiDAR points projected onto the camera frame.

    Args:
        projected_points (list): List of (x, y) coordinates of projected LiDAR points.
        frame_width (int): Width of the camera frame.
        frame_height (int): Height of the camera frame.
    """
    # 픽셀 좌표 분리
    x_coords = [x for x, y in projected_points if 0 <= x < frame_width and 0 <= y < frame_height]
    y_coords = [y for x, y in projected_points if 0 <= x < frame_width and 0 <= y < frame_height]

    if x_coords and y_coords:
        # X 축 스펙트럼 시각화
        plt.figure(figsize=(10, 5))
        plt.hist(x_coords, bins=50, color='blue', edgecolor='black', alpha=0.7)
        plt.title("LiDAR Projection Spectrum (X Axis)")
        plt.xlabel("X Pixel Coordinate")
        plt.ylabel("Frequency")
        plt.grid(True)
        plt.show()

        # Y 축 스펙트럼 시각화
        plt.figure(figsize=(10, 5))
        plt.hist(y_coords, bins=50, color='green', edgecolor='black', alpha=0.7)
        plt.title("LiDAR Projection Spectrum (Y Axis)")
        plt.xlabel("Y Pixel Coordinate")
        plt.ylabel("Frequency")
        plt.grid(True)
        plt.show()
    else:
        self.get_logger().info("No valid LiDAR points within the camera frame.")




def filter_noise_by_histogram(projected_points, distances, frame_height, min_frequency=5, num_bins=50):
    """
    Remove noise in projected points based on Y-coordinate histogram analysis.

    Args:
        projected_points (list): List of (x, y) pixel coordinates of projected points.
        distances (list): List of distances corresponding to projected points.
        frame_height (int): Height of the camera frame.
        min_frequency (int): Minimum frequency for a Y-coordinate bin to be considered valid.
        num_bins (int): Number of bins for the Y-coordinate histogram.

    Returns:
        filtered_points (list): List of filtered (x, y) points after removing noise.
        filtered_distances (list): List of distances corresponding to filtered points.
    """
    # Y 좌표 추출
    y_coords = [y for _, y in projected_points]

    # 히스토그램 계산
    hist, bin_edges = np.histogram(y_coords, bins=num_bins, range=(0, frame_height))

    # 유효한 Y 값의 범위 찾기
    valid_bins = np.where(hist >= min_frequency)[0]
    if len(valid_bins) == 0:
        return [], []  # 모든 데이터가 노이즈인 경우

    # 유효한 Y 값의 최소, 최대 범위 계산
    valid_min_y = bin_edges[valid_bins[0]]
    valid_max_y = bin_edges[valid_bins[-1]]

    # 필터링: 유효한 Y 범위 내의 점만 유지
    filtered_points = []
    filtered_distances = []

    for i, (point, dist) in enumerate(zip(projected_points, distances)):
        x, y = point
        if valid_min_y <= y <= valid_max_y:
            filtered_points.append((x, y))
            filtered_distances.append(dist)

    return filtered_points, filtered_distances
