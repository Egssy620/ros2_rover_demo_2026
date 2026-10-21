cmake_minimum_required(VERSION 3.8)
project(nedo_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

# ── livox_converter node ─────────────────────────────────────────────────────
add_executable(livox_converter src/livox_converter.cpp)
target_include_directories(livox_converter PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(livox_converter ${PCL_LIBRARIES})
ament_target_dependencies(livox_converter
  rclcpp sensor_msgs pcl_conversions)

# ── odom_imu_fuser node ──────────────────────────────────────────────────────
add_executable(odom_imu_fuser src/odom_imu_fuser.cpp)
ament_target_dependencies(odom_imu_fuser
  rclcpp sensor_msgs nav_msgs geometry_msgs)

# ── landmark_mapper_node ─────────────────────────────────────────────────────
add_executable(landmark_mapper_node src/landmark_mapper_node.cpp)
target_compile_features(landmark_mapper_node PRIVATE cxx_std_17)

# ── landmark_debug_node（デバッグ可視化: 強度フィルタ後 / ID認識点を配信）──────
add_executable(landmark_debug_node src/landmark_debug_node.cpp)
ament_target_dependencies(landmark_debug_node rclcpp sensor_msgs)

# ── landmark_bundle_adjustment ───────────────────────────────────────────────
# LIO-SAM 自己位置（固定）+ ランドマーク観測から Ceres でランドマーク地図を最適化
add_executable(landmark_bundle_adjustment src/landmark_bundle_adjustment.cpp)
target_compile_features(landmark_bundle_adjustment PRIVATE cxx_std_17)
target_link_libraries(landmark_bundle_adjustment Ceres::ceres)

install(TARGETS livox_converter odom_imu_fuser landmark_mapper_node
  landmark_bundle_adjustment landmark_debug_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
