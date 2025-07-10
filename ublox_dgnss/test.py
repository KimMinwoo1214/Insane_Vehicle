import math

def find_closest_slope_deg_rt(x, y, angle_deg, points):
    """
    RT 성능 최적화: 라디안 변환 최소화, projection distance 비교
    :param x: 기준점 x
    :param y: 기준점 y
    :param angle_deg: 직선 각도 (degree)
    :param points: [(px, py), ...]
    :return: (기울기, 가장 가까운 점 (px, py))
    """
    angle_rad = math.radians(angle_deg)
    dx = math.cos(angle_rad)
    dy = math.sin(angle_rad)

    # +x 정의역 보정
    if dx <= 0:
        dx *= -1
        dy *= -1

    dir_x, dir_y = dx, dy

    min_t = float('inf')
    closest_point = None

    for px, py in points:
        vec_x = px - x
        vec_y = py - y

        t = vec_x * dir_x + vec_y * dir_y  # dot product

        if t <= 0:
            continue  # 반대 방향

        if t < min_t:
            min_t = t
            closest_point = (px, py)

    if closest_point is None:
        return None, None

    delta_x = closest_point[0] - x
    if delta_x == 0:
        slope = float('inf')
    else:
        delta_y = closest_point[1] - y
        slope = delta_y / delta_x

    return slope, closest_point

# 사용 예시
if __name__ == "__main__":
    x, y = 0.0, 0.0
    angle_deg = 20.0
    points = [(2, 1), (3, 3), (5, 2), (-1, 2)]

    slope, point = find_closest_slope_deg_rt(x, y, angle_deg, points)
    print(f"가장 가까운 점: {point}, 기울기: {slope:.4f}")
