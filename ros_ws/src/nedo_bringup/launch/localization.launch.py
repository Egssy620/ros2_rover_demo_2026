import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument,GroupAction,LogInfo
from launch.conditions import IfCondition,UnlessCondition
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node,LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = LaunchDescription()


    # パス設定
    #lidar_launch_path = os.path.join(
    #    get_package_share_directory('hesai_ros_driver'),
    #    'launch',
    #    'start.py'
    #)
    package_share_dir = get_package_share_directory('nedo_bringup')
    default_front_calib_path = os.path.join(package_share_dir, 'config', 'front_camera_ost.yaml')
    default_left_calib_path = os.path.join(package_share_dir, 'config', 'left_camera_ost.yaml')
    default_right_calib_path = os.path.join(package_share_dir, 'config', 'right_camera_ost.yaml')
    #default_calib_path = os.path.join(
    #    get_package_share_directory('nedo_bringup'),
    #    'config',
    #    'ost.yaml'
    #)

    use_bag = LaunchConfiguration('use_bag')
    is_contour = LaunchConfiguration('is_contour')
    #calib_file =LaunchConfiguration('calib_file')
    front_calib_file = LaunchConfiguration('front_calib_file')
    left_calib_file = LaunchConfiguration('left_calib_file')
    right_calib_file = LaunchConfiguration('right_calib_file')

    declare_bag_cmd = DeclareLaunchArgument('use_bag',
                                            default_value='true',
                                            description='use bag or not')

    declare_contour_cmd = DeclareLaunchArgument('is_contour',
                                                default_value='true')

    #declare_calib_file_cmd = DeclareLaunchArgument('calib_file',
    #                                       default_value='file://'+default_calib_path,
    #                                       description='calibration yaml file')
    
    declare_front_calib_file_cmd = DeclareLaunchArgument(
        'front_calib_file',
        default_value='file://' + default_front_calib_path,
        description='Front camera calibration YAML file'
    )
    declare_left_calib_file_cmd = DeclareLaunchArgument(
        'left_calib_file',
        default_value='file://' + default_left_calib_path,
        description='Left camera calibration YAML file'
    )
    declare_right_calib_file_cmd = DeclareLaunchArgument(
        'right_calib_file',
        default_value='file://' + default_right_calib_path,
        description='Right camera calibration YAML file'
    )

    bag_namespace = 'collection'
    # グループ化された設定
    bag_group = GroupAction(
        condition=IfCondition(use_bag),
        actions=[
            #container for capture 
            Node(
                package='rclcpp_components',
                executable='component_container_mt',
                name='bag_capture',
                output='screen',
            ),
            #container for localization
            Node(
                package='rclcpp_components',
                executable='component_container_mt',
                name='bag_localization',
                output='screen',
            ),
            #load capture components
            LoadComposableNodes(
                target_container='bag_capture',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::CompressNode',
                        name='cap_comp',
                        namespace=bag_namespace
                    ),
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::CaminfoNode',
                        name='caminfo',
                        namespace=bag_namespace,
                        parameters=[
                            {'calib_file':front_calib_file}
                        ]
                    )
                ]
            ),
            LoadComposableNodes(
                condition=UnlessCondition(use_bag),
                target_container='bag_capture',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::DetectorNode',
                        name='point_detector',
                        namespace=bag_namespace
                    )
                ]
            ),
            LoadComposableNodes(
                condition=IfCondition(use_bag),
                target_container='bag_capture',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::ContourNode',
                        name='contour_detector',
                        namespace=bag_namespace
                    )
                ]
            ),

            #load localization components
            LoadComposableNodes(
                target_container='bag_localization',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::ProjectionNode',
                        name='projection',
                        namespace=bag_namespace
                    )
                ]
            )
        ]
    )

    robot_namespace = 'nedo'
    # グループ化された設定
    robot_group = GroupAction(
        condition=UnlessCondition(use_bag),
        actions=[
            #container for capture 
            Node(
                package='rclcpp_components',
                executable='component_container',
                name='robot_capture',
                output='screen',
            ),
            #container for localization
            Node(
                package='rclcpp_components',
                executable='component_container',
                name='robot_localization',
                output='screen',
            ),
            #load capture components
            LoadComposableNodes(
                target_container='robot_capture',
                composable_node_descriptions=[
                    # フロントカメラ情報
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::CaminfoNode',
                        name='front_caminfo',
                        namespace=robot_namespace + '/front',
                        parameters=[{'calib_file': front_calib_file}]
                    ),
                    # 左カメラ情報
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::CaminfoNode',
                        name='left_caminfo',
                        namespace=robot_namespace + '/left',
                        parameters=[{'calib_file': left_calib_file}]
                    ),
                    # 右カメラ情報
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::CaminfoNode',
                        name='right_caminfo',
                        namespace=robot_namespace + '/right',
                        parameters=[{'calib_file': right_calib_file}]
                    ),
                    #ComposableNode(
                    #    package='nedo_capture',
                    #    plugin='nedo_capture::NedoCapNode',
                    #    name='calib_cap',
                    #    namespace=robot_namespace
                    #),
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::CompressNode',
                        name='cap_comp',
                        namespace=robot_namespace
                    ),
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::CaminfoNode',
                        name='caminfo',
                        namespace=robot_namespace,
                        parameters=[
                            {'calib_file':front_calib_file}
                        ]
                    )
                ]
            ),
            LoadComposableNodes(
                condition=UnlessCondition(use_bag),
                target_container='robot_capture',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::DetectorNode',
                        name='point_detector',
                        namespace=robot_namespace
                    )
                ]
            ),
            LoadComposableNodes(
                condition=IfCondition(use_bag),
                target_container='robot_capture',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::ContourNode',
                        name='contour_detector',
                        namespace=robot_namespace
                    )
                ]
            ),

            #load localization components
            LoadComposableNodes(
                target_container='robot_localization',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::ProjectionNode',
                        name='projection',
                        namespace=robot_namespace
                    )
                ]
            ),
            #IncludeLaunchDescription(lidar_launch_path)
        ]
    )
    ld.add_action(LogInfo(msg=["is_contour: ", is_contour]))
    ld.add_action(declare_bag_cmd)
    ld.add_action(declare_contour_cmd)
    #ld.add_action(declare_calib_file_cmd)
    ld.add_action(declare_front_calib_file_cmd)
    ld.add_action(declare_left_calib_file_cmd)
    ld.add_action(declare_right_calib_file_cmd)
    ld.add_action(bag_group)
    ld.add_action(robot_group)
    
    return ld

#from launch import LaunchDescription
#from launch.actions import GroupAction
#from launch_ros.actions import Node, LoadComposableNodes
#from launch_ros.descriptions import ComposableNode
#from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch.actions import IncludeLaunchDescription
#from ament_index_python.packages import get_package_share_directory
#import os


#def generate_launch_description():
    ## パス設定
    #other_launch_file_path = os.path.join(
        #get_package_share_directory('another_package'),
        #'launch',
        #'other_launch_file.py'
    #)

    ## グループ化された設定
    #group_action = GroupAction(
        #actions=[
            ## コンポーネントノードのコンテナを定義
            #Node(
                #package='rclcpp_components',
                #executable='component_container',
                #name='my_container',
                #namespace='shared_namespace',
                #output='screen'
            #),
            ## コンポーネントノードをロード
            #LoadComposableNodes(
                #target_container='my_container',
                #composable_node_descriptions=[
                    #ComposableNode(
                        #package='my_package',
                        #plugin='my_package::Node1',
                        #name='node1',
                        #namespace='shared_namespace'
                    #),
                    #ComposableNode(
                        #package='my_package',
                        #plugin='my_package::Node2',
                        #name='node2',
                        #namespace='shared_namespace'
                    #)
                #]
            #),
            ## 他の Launch ファイルを実行
            #IncludeLaunchDescription(
                #PythonLaunchDescriptionSource(other_launch_file_path)
            #)
        #]
    #)

    ## LaunchDescription を返す
    #return LaunchDescription([group_action])
