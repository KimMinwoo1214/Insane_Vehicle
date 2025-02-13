import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # sllidar_ros2 패키지의 launch 파일 경로
    sllidar_launch_file = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'view_sllidar_s2_launch.py'
    )

    # laser_segmentation 패키지의 launch 파일 경로
    segmentation_launch_file = os.path.join(
        get_package_share_directory('laser_segmentation'),
        'launch',
        'segmentation.launch.py'
    )

    return LaunchDescription([
        # RPLiDAR S2 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_file),
            launch_arguments={}
        ),

        # Laser Segmentation 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(segmentation_launch_file),
            launch_arguments={}
        ),

        # 도로 차선 시각화 실행 (centroids_visualize)
        Node(
            package='centroids',
            executable='centroids_visualize',
            name='centroids_visualize',
            output='screen'
        )
    ])
