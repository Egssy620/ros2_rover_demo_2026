from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("mocap_localization_adapter"))
    default_parameters = package_share / "config" / "mocap_localization.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_parameters),
                description="モーションキャプチャ変換ノードのパラメータYAML",
            ),
            Node(
                package="mocap_localization_adapter",
                executable="mocap_localization_adapter_node",
                name="mocap_localization_adapter",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
