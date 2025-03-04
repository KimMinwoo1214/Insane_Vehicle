import numpy as np

def estimate_line(points):
    """
    추적 중인 좌표들로 직선을 추정.

    Args:
        points (list or np.ndarray): 추적 중인 좌표들. [[x1, y1], [x2, y2], ...]

    Returns:
        tuple: (기울기 m, y 절편 b)로 구성된 직선 방정식 파라미터.
    """
    points = np.array(points)
    if len(points) < 2:
        raise ValueError("At least two points are required to estimate a line.")

    x = points[:, 0]
    y = points[:, 1]

    # 기울기 m 계산
    N = len(points)
    m = (N * np.sum(x * y) - np.sum(x) * np.sum(y)) / (N * np.sum(x ** 2) - (np.sum(x) ** 2))

    # y 절편 b 계산
    b = (np.sum(y) - m * np.sum(x)) / N

    return m, b
