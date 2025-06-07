import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 패키지 이름
    pkg_name = 'md'

    # 왼쪽 모터 드라이버 노드 정의
    left_motor_node = Node(
        package=pkg_name,
        executable='md_robot_node',
        name='left_motor_driver',      # 1. 고유한 노드 이름
        namespace='left_motor',         # 2. 고유한 네임스페이스
        output='screen',
        emulate_tty=True,
        parameters=[
            {'serial_port': '/dev/ttyMotorLeft'}, # 3. 왼쪽 포트 지정
            
            # --- 공통 또는 왼쪽 모터용 파라미터 ---
            {'use_MDUI': 0},
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

    # 오른쪽 모터 드라이버 노드 정의
    right_motor_node = Node(
        package=pkg_name,
        executable='md_robot_node',
        name='right_motor_driver',     # 1. 고유한 노드 이름
        namespace='right_motor',        # 2. 고유한 네임스페이스
        output='screen',
        emulate_tty=True,
        parameters=[
            {'serial_port': '/dev/ttyMotorRight'}, # 3. 오른쪽 포트 지정

            # --- 공통 또는 오른쪽 모터용 파라미터 ---
            {'use_MDUI': 0},
            {'serial_baudrate': 57600},
            {'wheel_radius': 0.0935},
            {'wheel_length': 0.454},
            {'reduction': 30.0},
            {'reverse_direction': 1}, # 예: 오른쪽 모터가 반대로 장착된 경우 1로 설정
            {'maxrpm': 1000},
            {'enable_encoder': 0},
            {'encoder_PPR': 900},
            {'slow_start': 300},
            {'slow_down': 300},
        ]
    )

    # LaunchDescription 객체를 생성하고 두 개의 노드를 추가합니다.
    return LaunchDescription([
        left_motor_node,
        right_motor_node
    ])