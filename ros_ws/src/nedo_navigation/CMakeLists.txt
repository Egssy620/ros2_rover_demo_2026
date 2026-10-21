cmake_minimum_required(VERSION 3.8)
project(nedo_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# add executable
add_executable(nedo_controller src/nedo_controller.cpp)
ament_target_dependencies(nedo_controller rclcpp geometry_msgs)

#manual_cmd_vel_node
add_executable(manual_cmd_vel_node src/manual_cmd_vel_node.cpp)
ament_target_dependencies(manual_cmd_vel_node rclcpp geometry_msgs)
install(TARGETS manual_cmd_vel_node DESTINATION lib/${PROJECT_NAME})

#auto_cmd_vel_node_phase1.cpp
add_executable(auto_cmd_vel_node_phase1 src/auto_cmd_vel_node_phase1.cpp)
ament_target_dependencies(auto_cmd_vel_node_phase1 rclcpp geometry_msgs)
install(TARGETS auto_cmd_vel_node_phase1 DESTINATION lib/${PROJECT_NAME}) 

#auto_cmd_vel_node_feedback.cpp
add_executable(auto_cmd_vel_node_feedback src/auto_cmd_vel_node_feedback.cpp)
ament_target_dependencies(auto_cmd_vel_node_feedback rclcpp geometry_msgs nav_msgs)
install(TARGETS auto_cmd_vel_node_feedback DESTINATION lib/${PROJECT_NAME}) 

#auto_cmd_vel_node.cpp
add_executable(auto_cmd_vel_node src/auto_cmd_vel_node.cpp)
ament_target_dependencies(auto_cmd_vel_node rclcpp geometry_msgs)
install(TARGETS auto_cmd_vel_node DESTINATION lib/${PROJECT_NAME})

#rotate_cmd_vel_node.cpp
add_executable(rotate_cmd_vel_node src/rotate_cmd_vel_node.cpp)
ament_target_dependencies(rotate_cmd_vel_node rclcpp geometry_msgs)
install(TARGETS rotate_cmd_vel_node DESTINATION lib/${PROJECT_NAME})

#cmd_vel_to_twist_node.cpp : /cmd_vel(Twist) -> /cmd_vel_cov(TwistWithCovarianceStamped)
# robot_localization(EKF) の疑似速度入力 (LM途切れ時のデッドレコニング源)
add_executable(cmd_vel_to_twist_node src/cmd_vel_to_twist_node.cpp)
ament_target_dependencies(cmd_vel_to_twist_node rclcpp geometry_msgs)
install(TARGETS cmd_vel_to_twist_node DESTINATION lib/${PROJECT_NAME})

#rover_odo_to_twist_node.cpp : /rover_odo(TwistStamped) -> /rover_odo_cov(TwistWithCovarianceStamped)
# robot_localization(EKF) の車輪オドメトリ入力 (twistN は TwistWithCovarianceStamped のみ受付)
add_executable(rover_odo_to_twist_node src/rover_odo_to_twist_node.cpp)
ament_target_dependencies(rover_odo_to_twist_node rclcpp geometry_msgs)
install(TARGETS rover_odo_to_twist_node DESTINATION lib/${PROJECT_NAME})

#pickup_planner_node.cpp : 把持可能範囲の点群から次のピックアップ位置を決め /goal_pose に送る
add_executable(pickup_planner_node src/pickup_planner_node.cpp)
ament_target_dependencies(pickup_planner_node rclcpp sensor_msgs geometry_msgs std_msgs std_srvs visualization_msgs tf2 tf2_ros)
install(TARGETS pickup_planner_node DESTINATION lib/${PROJECT_NAME})

#simple_goal_follower_node.cpp : 動作確認用の簡易移動 (/goal_pose → /cmd_vel)
add_executable(simple_goal_follower_node src/simple_goal_follower_node.cpp)
ament_target_dependencies(simple_goal_follower_node rclcpp geometry_msgs)
install(TARGETS simple_goal_follower_node DESTINATION lib/${PROJECT_NAME})

#pickup_key.py : pickup_planner_node をキーボードで操作 (Enter=次へ s=飛ばす r=リセット)
install(PROGRAMS scripts/pickup_key.py DESTINATION lib/${PROJECT_NAME})

#autonomous_sequence_node.cpp (build only if source exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/autonomous_sequence_node.cpp)
  add_executable(autonomous_sequence_node src/autonomous_sequence_node.cpp)
  ament_target_dependencies(autonomous_sequence_node rclcpp geometry_msgs std_msgs)
  install(TARGETS autonomous_sequence_node DESTINATION lib/${PROJECT_NAME})
endif()

#simple_autonomous_node.cpp (build only if source exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/simple_autonomous_node.cpp)
  add_executable(simple_autonomous_node src/simple_autonomous_node.cpp)
  ament_target_dependencies(simple_autonomous_node rclcpp geometry_msgs std_msgs)
  install(TARGETS simple_autonomous_node DESTINATION lib/${PROJECT_NAME})
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install targets (only those guaranteed to exist here)
install(TARGETS
  nedo_controller
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)
#install(PROGRAMS
  #test/test.bash
  #test/test_gui.bash
  #DESTINATION lib/${PROJECT_NAME}
#)

#ament_auto_package(
  #INSTALL_TO_SHARE
  #config
  #launch
  #test
#)

ament_package()
