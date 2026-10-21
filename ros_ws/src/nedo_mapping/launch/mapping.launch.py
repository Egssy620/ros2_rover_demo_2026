from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("nedo_mapping")
    # ── Launch arguments ──────────────────────────────────────────────────
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=str(Path(pkg_share) / "config" / "params_hesai.yaml"),
        description="Path to LIO-SAM parameter YAML",
    )

    save_dir_arg = DeclareLaunchArgument(
        "save_dir",
        default_value=PathJoinSubstitution([EnvironmentVariable("HOME"), "lio_sam_output", ""]),
        description="Directory for PCD export (trailing slash required by LIO-SAM)",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (bag) clock. Set true when replaying bags with --clock.",
    )

    use_odom_fusion_arg = DeclareLaunchArgument(
        "use_odom_fusion",
        default_value="true",
        description="Fuse /odom with IMU via robot_localization EKF. "
                    "When false, odom_imu_fuser acts as a transparent relay "
                    "(LIO-SAM ignores orientation until EKF data arrives).",
    )

    ekf_odom_topic_arg = DeclareLaunchArgument(
        "ekf_odom_topic",
        default_value="/odom",
        description="Wheel odometry topic fed into EKF (odom0)",
    )

    ekf_imu_topic_arg = DeclareLaunchArgument(
        "ekf_imu_topic",
        default_value="/livox/imu_3JEDM4V001DY881",
        description="IMU topic fed into EKF (imu0)",
    )

    imu_raw_topic_arg = DeclareLaunchArgument(
        "imu_raw_topic",
        default_value="/livox/imu_3JEDM4V001DY881",
        description="Raw IMU topic for odom_imu_fuser (imu_raw remapping)",
    )

    params_file = LaunchConfiguration("params_file")
    save_dir = LaunchConfiguration("save_dir")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_odom_fusion = LaunchConfiguration("use_odom_fusion")
    ekf_odom_topic = LaunchConfiguration("ekf_odom_topic")
    ekf_imu_topic = LaunchConfiguration("ekf_imu_topic")
    imu_raw_topic = LaunchConfiguration("imu_raw_topic")

    sim_time_param = {"use_sim_time": use_sim_time}
    save_override = {"savePCDDirectory": save_dir}

    # ── livox_converter: line→ring, timestamp→time ────────────────────────
    livox_converter = Node(
        package="nedo_mapping",
        executable="livox_converter",
        name="livox_converter",
        parameters=[sim_time_param],
        remappings=[
            ("input",  "/lidar_points"),
            ("output", "/livox/lidar_lio_sam"),
        ],
        output="screen",
    )

    # ── EKF: /odom + /livox/imu → /odometry/filtered (50 Hz) ────────────
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[
            str(Path(pkg_share) / "config" / "ekf_mapping.yaml"),
            {"odom0": ekf_odom_topic, "imu0": ekf_imu_topic},
            sim_time_param,
        ],
        condition=IfCondition(use_odom_fusion),
        output="screen",
    )

    # ── odom_imu_fuser: inject EKF orientation → /imu/fused (200 Hz) ────
    # Publishes with orientation_covariance[0]=-1 (unknown) until EKF arrives,
    # so LIO-SAM works correctly even if the EKF has not yet initialised.
    odom_imu_fuser = Node(
        package="nedo_mapping",
        executable="odom_imu_fuser",
        name="odom_imu_fuser",
        parameters=[sim_time_param],
        remappings=[
            ("imu_raw",       imu_raw_topic),
            ("odom_filtered", "/odometry/filtered"),
            ("imu_fused",     "/imu/fused"),
        ],
        output="screen",
    )

    # ── LIO-SAM nodes ────────────────────────────────────────────────────
    imu_preintegration = Node(
        package="lio_sam",
        executable="lio_sam_imuPreintegration",
        name="lio_sam_imuPreintegration",
        parameters=[params_file, save_override, sim_time_param],
        output="screen",
    )

    image_projection = Node(
        package="lio_sam",
        executable="lio_sam_imageProjection",
        name="lio_sam_imageProjection",
        parameters=[params_file, save_override, sim_time_param],
        output="screen",
    )

    feature_extraction = Node(
        package="lio_sam",
        executable="lio_sam_featureExtraction",
        name="lio_sam_featureExtraction",
        parameters=[params_file, save_override, sim_time_param],
        output="screen",
    )

    map_optimization = Node(
        package="lio_sam",
        executable="lio_sam_mapOptimization",
        name="lio_sam_mapOptimization",
        parameters=[params_file, save_override, sim_time_param],
        output="screen",
    )

    # ── RViz2 (optional) ─────────────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[sim_time_param],
        arguments=["-d", str(Path(pkg_share) / "config" / "rviz2.rviz")],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription([
        params_file_arg,
        save_dir_arg,
        use_rviz_arg,
        use_sim_time_arg,
        use_odom_fusion_arg,
        ekf_odom_topic_arg,
        ekf_imu_topic_arg,
        imu_raw_topic_arg,
        livox_converter,
        ekf_node,
        odom_imu_fuser,
        imu_preintegration,
        image_projection,
        feature_extraction,
        map_optimization,
        rviz,
    ])
