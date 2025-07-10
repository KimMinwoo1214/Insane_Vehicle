import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 조이스틱 입력 노드 실행
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # SerialBridge (아두이노와 시리얼 통신) 실행
        # 조이스틱을 통한 ROS2 명령 생성 노드 실행
        Node(
            package='mega_con',
            executable='joy_con',
            name='joy_control',
            output='log'
        ),
    ])
