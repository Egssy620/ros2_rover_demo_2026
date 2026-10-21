from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nedo_navigation',
            executable='manual_cmd_vel_node',
            name='manual_cmd_vel_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                # 必要に応じてパラメータを追加
            }]
        )
    ])
