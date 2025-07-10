from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1) 토픽 이름을 런치 아규먼트로 선언 (기본값: /rslidar_points)
    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='/rslidar_points',
        description='Input PointCloud2 topic from RS-LiDAR'
    )

    # 2) 패키지 공유 디렉토리 경로 가져오기
    sdk_share = get_package_share_directory('rslidar_sdk')
    config_file = os.path.join(sdk_share, 'config', 'config.yaml')
    rviz_config = os.path.join(sdk_share, 'rviz', 'rviz2.rviz')

    return LaunchDescription([
        topic_arg,

        # 3) RS-LiDAR SDK 드라이버 노드
        Node(
            package='rslidar_sdk',
            executable='rslidar_sdk_node',    # 실제 설치된 실행 파일 이름
            name='rslidar_driver',
            output='screen',
            parameters=[config_file],         # YAML 파일 경로
        ),

        # 4) 콘 검출 노드
        Node(
            package='cone_detection_lidar',
            executable='detection_simple',
            name='cone_detector',
            output='screen',
            parameters=[
                {'cloud_in_topic': LaunchConfiguration('topic')},
                {'points_out_topic': '/clustered_cones'},
                {'marker_out_topic': '/cone_markers'},
                {'eps': 0.3},
                {'cluster_points_min': 10},
                {'cluster_points_max': 25000},
                {'minX': 0.0},
                {'maxX': 5.0},
                {'minY': -5.0},
                {'maxY': 5.0},
                {'minZ': -0.4},
                {'maxZ':  0.3},
                {'verbose1': False},
                {'verbose2': True},
            ],
        ),

        # 5) RViz 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])

