cmake_minimum_required(VERSION 3.8)
project(odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(antrobotics_ros_can_interfaces REQUIRED)
find_package(antrobotics_can_commands_interfaces REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(realtime_tools REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/odometry_ros.cpp)
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  rclcpp_components
  rcpputils
  nav_msgs
  std_msgs
  std_srvs
  antrobotics_ros_can_interfaces
  antrobotics_can_commands_interfaces
  tf2_ros
  diff_drive_controller
  realtime_tools
)

rclcpp_components_register_nodes(${PROJECT_NAME} "OdometryROS")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS 
    ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include/${PROJECT_NAME})

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
