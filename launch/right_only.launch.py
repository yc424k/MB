from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='md',
            executable='md_robot_node',
            name='right_motor_driver',
            namespace='right_motor',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'serial_port': '/dev/ttyMotorRight'},
                {'use_MDUI': 0},
                {'serial_baudrate': 57600},
                {'wheel_length': 0.454},
                {'reduction': 30.0},
                {'reverse_direction': 1},
                {'maxrpm': 1000},
                {'enable_encoder': 0},
                {'encoder_PPR': 900},
                {'slow_start': 300},
                {'slow_down': 300},
            ]
        )
    ])