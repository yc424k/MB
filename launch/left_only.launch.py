from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='md',
            executable='md_robot_node',
            name='left_motor_driver',
            namespace='left_motor',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'serial_port': '/dev/ttyMotorLeft'},
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
    ])