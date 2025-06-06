import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 패키지 이름
    pkg_name = 'md'

    # 실행할 노드와 파라미터를 정의합니다.
    md_robot_node = Node(
        package=pkg_name,
        executable='md_robot_node',
        name='md_robot_node',
        output='screen',
        emulate_tty=True, # ROS1의 output="screen"과 유사한 동작을 위해 추가
        parameters=[
            {'use_MDUI': 0},
            {'serial_port': 'ttyUSB0'},
            {'serial_baudrate': 57600},
            {'wheel_radius': 0.0935},
            {'wheel_length': 0.454},
            {'reduction': 30.0},
            {'reverse_direction': 0},
            {'maxrpm': 1000},
            {'enable_encoder': 0},
            {'encoder_PPR': 900},
            {'slow_start': 300},
            {'slow_down': 300},
        ]
    )

    # LaunchDescription 객체를 생성하고 노드를 추가합니다.
    ld = LaunchDescription()
    ld.add_action(md_robot_node)
    
    return ld