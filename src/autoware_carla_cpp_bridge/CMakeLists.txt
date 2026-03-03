cmake_minimum_required(VERSION 3.8)
project(autoware_carla_cpp_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(aw_carla_bridge_node
  src/aw_carla_bridge_node.cpp
  src/aw_carla_bridge.cpp
)

target_include_directories(aw_carla_bridge_node PUBLIC include)

ament_target_dependencies(aw_carla_bridge_node
  rclcpp
  std_msgs
  diagnostic_msgs
)

install(TARGETS aw_carla_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(lidar_bridge_node
  src/lidar_bridge.cpp
)

target_include_directories(lidar_bridge_node PUBLIC include)

ament_target_dependencies(lidar_bridge_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
)

install(TARGETS lidar_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(camera_bridge_node
  src/camera_bridge.cpp
)

target_include_directories(camera_bridge_node PUBLIC include)

ament_target_dependencies(camera_bridge_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
)

install(TARGETS camera_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
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

ament_package()
