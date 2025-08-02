cmake_minimum_required(VERSION 3.8)
project(dwa_motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "action/DwaPlanner.action"
)

add_executable(dwa_action_server src/dwa_action_server.cpp)
add_executable(dwa_action_client src/dwa_action_client.cpp)

ament_target_dependencies(dwa_action_server
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

ament_target_dependencies(dwa_action_client rclcpp rclcpp_action)

rosidl_target_interfaces(dwa_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(dwa_action_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  dwa_action_server
  dwa_action_client

  DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY launch 
        DESTINATION share/${PROJECT_NAME}
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
