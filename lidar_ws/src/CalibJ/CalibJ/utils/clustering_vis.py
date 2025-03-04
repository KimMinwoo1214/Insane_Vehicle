import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import builtin_interfaces.msg
import struct

def points_to_pointcloud2(labels, points, frame_id="laser"):
    """
    클러스터링된 점들을 PointCloud2 메시지로 변환하며, 라벨별로 색상을 지정.

    Args:
        labels (np.ndarray): 각 점의 클러스터 라벨 (-1은 노이즈)
        points (np.ndarray): Nx2 배열 (x, y 좌표)
        frame_id (str): PointCloud2 메시지의 좌표계

    Returns:
        PointCloud2: ROS2 메시지
    """
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = builtin_interfaces.msg.Time()  # 현재 ROS 시간

    # 포인트 필드 정의 (x, y, z, rgb)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16  # 각 포인트의 크기 (x=4 + y=4 + z=4 + rgb=4 bytes)
    msg.row_step = msg.point_step * len(points)  # 한 줄의 크기

    # 클러스터별로 색상을 지정
    unique_labels = np.unique(labels)
    colors = generate_colors(len(unique_labels))  # 고유 라벨 개수만큼 색상 생성

    cloud_data = []
    for point, label in zip(points, labels):
        x, y = point
        z = 0.0  # 2D 포인트이므로 z=0으로 설정
        color = colors[label] if label != -1 else (128, 128, 128)  # 노이즈(-1)는 회색
        rgb = pack_rgb(*color)

        # 포인트 데이터 (x, y, z, rgb)
        cloud_data.append(struct.pack('fffI', x, y, z, rgb))

    # 메시지 데이터 직렬화
    msg.data = b''.join(cloud_data)
    msg.height = 1
    msg.width = len(points)
    msg.is_dense = True  # 유효하지 않은 데이터 없음

    return msg


def generate_colors(num_colors):
    """
    고유 라벨의 개수만큼 색상을 생성.

    Args:
        num_colors (int): 생성할 색상의 개수

    Returns:
        dict: {라벨: (R, G, B)}
    """
    np.random.seed(0)  # 고정된 색상을 위해 시드 설정
    colors = np.random.randint(0, 255, size=(num_colors, 3))
    return {i: tuple(color) for i, color in enumerate(colors)}


def pack_rgb(r, g, b):
    """
    RGB 색상을 32비트 정수로 변환.

    Args:
        r (int): Red (0-255)
        g (int): Green (0-255)
        b (int): Blue (0-255)

    Returns:
        int: 32비트 RGB 값
    """
    return (r << 16) | (g << 8) | b


