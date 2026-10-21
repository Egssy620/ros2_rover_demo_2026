cmake_minimum_required(VERSION 3.8)
project(nedo_master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(master_node src/master_node.cpp)
ament_target_dependencies(master_node rclcpp geometry_msgs std_msgs std_srvs)

install(TARGETS master_node DESTINATION lib/${PROJECT_NAME})
# 機構部 (マイコン) の簡易ノード。依頼に自動で答える
install(PROGRAMS scripts/mechanism_node.py DESTINATION lib/${PROJECT_NAME})
# マスターのログだけを別のターミナルに出す
install(PROGRAMS scripts/master_log.py DESTINATION lib/${PROJECT_NAME})
# UI (仮): マスターと各ノードのトピックを読んで表示する
install(PROGRAMS scripts/master_ui.py DESTINATION lib/${PROJECT_NAME})

ament_package()
