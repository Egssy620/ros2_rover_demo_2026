cmake_minimum_required(VERSION 3.8)
project(nedo_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang") 
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io segmentation kdtree search)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vehicle_interface REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(message_filters)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

pkg_check_modules(LIBUDEV REQUIRED libudev)
include_directories(${LIBUDEV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/sorter_localization)
include_directories(${PCL_INCLUDE_DIRS})

add_library(projection_util SHARED src/projection_util.cpp)
# detection_util (optional - source may be missing in this snapshot)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/detection_utils.cpp)
  add_library(detection_util SHARED src/detection_utils.cpp)
endif()

# projection_util

target_include_directories(projection_util
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(projection_util
  geometry_msgs
  sensor_msgs
  pcl_conversions
  tf2_ros
  OpenCV
  vehicle_interface
)


# projection_node_component
add_library(projection_node_component SHARED src/projection_node.cpp)
target_link_libraries(projection_node_component projection_util)
target_compile_features(projection_node_component PUBLIC c_std_99 cxx_std_17)
target_include_directories(projection_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(projection_node_component
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  std_msgs
  pcl_conversions
  geometry_msgs
  tf2_ros
  rclcpp_components
  vehicle_interface
)
target_compile_definitions(projection_node_component PRIVATE "NEDO_CAPTURE_BUILDING_LIBRARY")
rclcpp_components_register_node(projection_node_component PLUGIN "nedo_localization::ProjectionNode" EXECUTABLE projection_node)

# lidar_cam_tf_component (requires detection_util)
if(TARGET detection_util)
add_library(lidar_cam_tf_component SHARED src/lidar_cam_tf_node.cpp)
target_link_libraries(lidar_cam_tf_component detection_util)
target_compile_features(lidar_cam_tf_component PUBLIC c_std_99 cxx_std_17)
target_include_directories(lidar_cam_tf_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(lidar_cam_tf_component
  rclcpp
  ament_index_cpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  rclcpp_components
)
target_compile_definitions(lidar_cam_tf_component PRIVATE "NEDO_LOCALIZATION_BUILDING_LIBRARY")
rclcpp_components_register_node(lidar_cam_tf_component PLUGIN "nedo_localization::LidarCamTF" EXECUTABLE lidar_cam_tf)
endif()

# localizer_component
add_library(localizer_component SHARED src/localizer.cpp)
target_compile_features(localizer_component PUBLIC c_std_99 cxx_std_17)
target_include_directories(localizer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(localizer_component
  rclcpp
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  rclcpp_components
  vehicle_interface
)
target_compile_definitions(localizer_component PRIVATE "NEDO_LOCALIZATION_BUILDING_LIBRARY")
rclcpp_components_register_node(localizer_component PLUGIN "nedo_localization::Localizer" EXECUTABLE localizer_node)

# landmark_marker_publisher_component (optional)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/landmark_marker_publisher.cpp)
  add_library(landmark_marker_publisher_component SHARED src/landmark_marker_publisher.cpp)
  target_compile_features(landmark_marker_publisher_component PUBLIC c_std_99 cxx_std_17)
  target_include_directories(landmark_marker_publisher_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(landmark_marker_publisher_component
    rclcpp
    geometry_msgs
    visualization_msgs
    rclcpp_components
    vehicle_interface
  )
  target_compile_definitions(landmark_marker_publisher_component PRIVATE "NEDO_LOCALIZATION_BUILDING_LIBRARY")
  rclcpp_components_register_node(landmark_marker_publisher_component PLUGIN "nedo_localization::LandmarkMarkerPublisher" EXECUTABLE landmark_marker_publisher)
endif()

# landmark_circle_filter_node (optional)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/landmark_circle_filter_node.cpp)
  add_executable(landmark_circle_filter_node src/landmark_circle_filter_node.cpp)
  target_include_directories(landmark_circle_filter_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(landmark_circle_filter_node rclcpp sensor_msgs geometry_msgs vehicle_interface)
endif()

# hesai_region_filter_node (optional)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/hesai_region_filter_node.cpp)
  add_executable(hesai_region_filter_node src/hesai_region_filter_node.cpp)
  target_include_directories(hesai_region_filter_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(hesai_region_filter_node rclcpp sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs tf2_sensor_msgs vehicle_interface)
  target_link_libraries(hesai_region_filter_node ${PCL_LIBRARIES})
endif()

# scrap_pile_skirt_node (optional): スクラップ山の麓・最近傍点・把持可能範囲・接近限界
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/scrap_pile_skirt_node.cpp)
  add_executable(scrap_pile_skirt_node src/scrap_pile_skirt_node.cpp)
  target_include_directories(scrap_pile_skirt_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(scrap_pile_skirt_node rclcpp sensor_msgs geometry_msgs std_msgs std_srvs visualization_msgs tf2 tf2_ros tf2_sensor_msgs)
  target_link_libraries(scrap_pile_skirt_node ${PCL_LIBRARIES})
endif()

# highest_point_node (optional)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/highest_point_node.cpp)
  add_executable(highest_point_node src/highest_point_node.cpp)
  target_include_directories(highest_point_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(highest_point_node rclcpp sensor_msgs geometry_msgs)
endif()

# integrated_landmark_approach_node (optional)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/integrated_landmark_approach_node.cpp)
  add_executable(integrated_landmark_approach_node src/integrated_landmark_approach_node.cpp)
  target_include_directories(integrated_landmark_approach_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(integrated_landmark_approach_node rclcpp sensor_msgs geometry_msgs vehicle_interface)
endif()

# landmark_map_manager_node (optional)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/landmark_map_manager_node.cpp)
  add_executable(landmark_map_manager_node src/landmark_map_manager_node.cpp)
  target_include_directories(landmark_map_manager_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(landmark_map_manager_node rclcpp sensor_msgs geometry_msgs vehicle_interface tf2 tf2_geometry_msgs tf2_ros)
endif()

# matched_marker_collector_node (optional)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/matched_marker_collector_node.cpp)
  add_executable(matched_marker_collector_node src/matched_marker_collector_node.cpp)
  target_include_directories(matched_marker_collector_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(matched_marker_collector_node rclcpp geometry_msgs vehicle_interface)
endif()

# matched_marker_aggregator_node (optional)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/matched_marker_aggregator_node.cpp)
  add_executable(matched_marker_aggregator_node src/matched_marker_aggregator_node.cpp)
  target_include_directories(matched_marker_aggregator_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(matched_marker_aggregator_node rclcpp geometry_msgs vehicle_interface tf2_ros tf2_geometry_msgs tf2)
endif()

# landmark_map_visualizer_node
add_executable(landmark_map_visualizer_node src/landmark_map_visualizer_node.cpp)
target_include_directories(landmark_map_visualizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(landmark_map_visualizer_node rclcpp visualization_msgs geometry_msgs std_msgs vehicle_interface)

# landmark_map_server_node (publish LM_* from params.yaml as topic)
add_executable(landmark_map_server_node src/landmark_map_server_node.cpp)
target_include_directories(landmark_map_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(landmark_map_server_node rclcpp geometry_msgs std_msgs vehicle_interface)

# LiDAR TF Publisher Node
add_executable(lidar_tf_publisher_node src/lidar_tf_publisher_node.cpp)
target_include_directories(lidar_tf_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lidar_tf_publisher_node rclcpp tf2_ros tf2 geometry_msgs)
#pose_converter_node (optional)
add_executable(pose_converter src/pose_converter.cpp)
ament_target_dependencies(pose_converter rclcpp geometry_msgs)

# imu_restamp_node : /livox/imu(デバイス時計) を now() で再スタンプし EKF 入力の
# クロック領域をホスト時刻に揃える
add_executable(imu_restamp_node src/imu_restamp_node.cpp)
ament_target_dependencies(imu_restamp_node rclcpp sensor_msgs)

# cylinder_landmark_detector_bin_node
# BIN方式のランドマーク検出器を /matched_marker に出力する
# (lidar_practice3 からの移植版。OT128 移行時は launch のパラメータで切替)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/cylinder_landmark_detector_bin_node.cpp)
  add_executable(cylinder_landmark_detector_bin_node
    src/cylinder_landmark_detector_bin_node.cpp)
  target_include_directories(cylinder_landmark_detector_bin_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(cylinder_landmark_detector_bin_node
    rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs
    tf2 tf2_ros tf2_geometry_msgs vehicle_interface)
endif()

# realtime_visualizer_component (optional)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/realtime_visualizer_node.cpp)
  add_library(realtime_visualizer_component SHARED src/realtime_visualizer_node.cpp)
  target_compile_features(realtime_visualizer_component PUBLIC c_std_99 cxx_std_17)
  target_include_directories(realtime_visualizer_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(realtime_visualizer_component
    rclcpp
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2_ros
    tf2_geometry_msgs
    tf2
    rclcpp_components
  )
  target_compile_definitions(realtime_visualizer_component PRIVATE "NEDO_LOCALIZATION_BUILDING_LIBRARY")
  rclcpp_components_register_node(realtime_visualizer_component PLUGIN "nedo_localization::RealTimeVisualizer" EXECUTABLE realtime_visualizer_node)
endif()

# installs
install(
  DIRECTORY include/
  DESTINATION include
)

# Collect component libs to install/export (only those that were built)
# NOTE: contour/detector/contour_matcher/data_sync の各 component および
#       detection_util / lidar_cam_tf_component は本スナップショットで
#       ソース未提供のため if(TARGET ...) で条件追加する。
set(_component_libs
  projection_util
  projection_node_component
  localizer_component
)
if(TARGET detection_util)
  list(APPEND _component_libs detection_util)
endif()
if(TARGET lidar_cam_tf_component)
  list(APPEND _component_libs lidar_cam_tf_component)
endif()
if(TARGET landmark_marker_publisher_component)
  list(APPEND _component_libs landmark_marker_publisher_component)
endif()
if(TARGET realtime_visualizer_component)
  list(APPEND _component_libs realtime_visualizer_component)
endif()

install(
  TARGETS ${_component_libs}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Collect executables to install (only those that were built)
set(_exec_targets)
if(TARGET landmark_circle_filter_node)
  list(APPEND _exec_targets landmark_circle_filter_node)
endif()
if(TARGET hesai_region_filter_node)
  list(APPEND _exec_targets hesai_region_filter_node)
endif()
if(TARGET scrap_pile_skirt_node)
  list(APPEND _exec_targets scrap_pile_skirt_node)
endif()
if(TARGET highest_point_node)
  list(APPEND _exec_targets highest_point_node)
endif()
if(TARGET integrated_landmark_approach_node)
  list(APPEND _exec_targets integrated_landmark_approach_node)
endif()
if(TARGET landmark_map_manager_node)
  list(APPEND _exec_targets landmark_map_manager_node)
endif()
if(TARGET matched_marker_collector_node)
  list(APPEND _exec_targets matched_marker_collector_node)
endif()
if(TARGET matched_marker_aggregator_node)
  list(APPEND _exec_targets matched_marker_aggregator_node)
endif()
if(TARGET landmark_map_visualizer_node)
  list(APPEND _exec_targets landmark_map_visualizer_node)
endif()
if(TARGET landmark_map_server_node)
  list(APPEND _exec_targets landmark_map_server_node)
endif()
if(TARGET lidar_tf_publisher_node)
  list(APPEND _exec_targets lidar_tf_publisher_node)
endif()
if(TARGET pose_converter)
  list(APPEND _exec_targets pose_converter)
endif()
if(TARGET imu_restamp_node)
  list(APPEND _exec_targets imu_restamp_node)
endif()
if(TARGET cylinder_landmark_detector_bin_node)
  list(APPEND _exec_targets cylinder_landmark_detector_bin_node)
endif()

if(_exec_targets)
  install(
    TARGETS ${_exec_targets}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Config.cmake
  NAMESPACE nedo_localization::
  DESTINATION share/${PROJECT_NAME}/cmake
)

#install(DIRECTORY launch/
#  DESTINATION share/${PROJECT_NAME}/launch
#)
# Install RViz configuration files
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
  install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(OpenCV vehicle_interface)
# Export only the libraries that exist
set(_export_libs
  projection_util
  projection_node_component
  localizer_component
)
if(TARGET detection_util)
  list(APPEND _export_libs detection_util)
endif()
if(TARGET lidar_cam_tf_component)
  list(APPEND _export_libs lidar_cam_tf_component)
endif()
if(TARGET landmark_marker_publisher_component)
  list(APPEND _export_libs landmark_marker_publisher_component)
endif()
if(TARGET realtime_visualizer_component)
  list(APPEND _export_libs realtime_visualizer_component)
endif()

ament_export_libraries(${_export_libs})
ament_export_targets(export_${PROJECT_NAME})
ament_package()
