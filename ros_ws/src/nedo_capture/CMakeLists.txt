cmake_minimum_required(VERSION 3.8)
project(nedo_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# for nedo_cap_node component...
add_library(nedo_cap_node_component src/nedo_cap_node.cpp)

target_compile_features(nedo_cap_node_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(nedo_cap_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  nedo_cap_node_component
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  rclcpp_components
)

target_compile_definitions(nedo_cap_node_component PRIVATE "NEDO_CAPTURE_BUILDING_LIBRARY")

rclcpp_components_register_node(
  nedo_cap_node_component
  PLUGIN "nedo_capture::NedoCapNode"
  EXECUTABLE nedo_cap_node
  )

# for compress node component...
add_library(compress_node_component src/compress_node.cpp)

target_compile_features(compress_node_component PUBLIC c_std_99 cxx_std_17)

target_include_directories(compress_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  compress_node_component
  rclcpp
  sensor_msgs
  image_transport
  rclcpp_components
)

target_compile_definitions(compress_node_component PRIVATE "NEDO_CAPTURE_BUILDING_LIBRARY")

rclcpp_components_register_node(
  compress_node_component
  PLUGIN "nedo_capture::CompressNode"
  EXECUTABLE compress_node
  )

# for caminfo_node component...
add_library(caminfo_node_component src/caminfo_node.cpp)

target_compile_features(caminfo_node_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(caminfo_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  caminfo_node_component
  rclcpp
  sensor_msgs
  camera_info_manager
  rclcpp_components
)

target_compile_definitions(caminfo_node_component PRIVATE "NEDO_CAPTURE_BUILDING_LIBRARY")

rclcpp_components_register_node(
  caminfo_node_component
  PLUGIN "nedo_capture::CaminfoNode"
  EXECUTABLE caminfo_node
  )


# install libralies
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS nedo_cap_node_component compress_node_component caminfo_node_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_export_include_directories(
  include
)
ament_export_libraries(
  nedo_cap_node_component
  compress_node_component
  caminfo_node_component
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
