from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, LoadComposableNodes
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    print("start camera calibration ...")
    print("Usage:")
    print("  Front camera: ros2 launch nedo_bringup calib_cam_launch.py")
    print("  Right camera: ros2 launch nedo_bringup calib_cam_launch.py device_file:=/dev/nedo_cam_right frame_id:=right_camera_frame")
    print("Current device mapping:")
    print("  Front camera (udev): /dev/nedo_cam_front (/dev/video1)")
    print("  Right camera (udev): /dev/nedo_cam_right (/dev/video7)")
    print("  Resolution: 4K (3840x2160) for calibration")
    
    device_file = LaunchConfiguration('device_file')
    frame_id = LaunchConfiguration('frame_id')
    
    declare_device_file_cmd = DeclareLaunchArgument(
        'device_file',
        default_value='/dev/nedo_cam_front',
        description='Camera device file (use /dev/nedo_cam_front or /dev/nedo_cam_right)'
    )
    declare_frame_id_cmd = DeclareLaunchArgument(
        'frame_id',
        default_value='front_camera_frame',
        description='Camera frame ID (front_camera_frame or right_camera_frame)'
    )
    
    container_name = 'test_container'
    ld = LaunchDescription()
    container = Node(
        name= container_name,
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        )
    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='nedo_capture',
                plugin='nedo_capture::NedoCapNode',
                name='calib_cap',
                namespace='calibration',
                parameters=[
                    {'device': device_file},
                    {'frame_id': frame_id},
                    {'pipeline': PythonExpression([
                        "'v4l2src device=", device_file, 
                        " io-mode=2 ! image/jpeg,width=3840,height=2160,framerate=30/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true'"
                    ])}
                ]
            )
        ]
    )
    calib_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        output='screen',
        arguments=[
            '--size','7x9',        # 7x9 checkerboard (inner corners)
            '--square', '0.020',   # 20mm square size
            '--no-service-check'   # Added to skip service check
        ],
        parameters=[
            {'camera':'/nedo_camera'}
        ],
        remappings=[
            ('image','/calibration/capture'),
        ]
    )
    ld.add_action(declare_device_file_cmd)
    ld.add_action(declare_frame_id_cmd)
    ld.add_action(container)
    ld.add_action(load_composable_nodes)
    ld.add_action(calib_node)

    return ld


