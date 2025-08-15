cmake_minimum_required(VERSION 3.5)
project(diablo_pathtracking)

find_package(rosidl_default_generators REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionCtrl.msg"
  "msg/MovementCtrlData.msg"
  "msg/MovementCtrlMode.msg"
  DEPENDENCIES builtin_interfaces
)

add_executable(control_node src/diablo_track_node.cpp)
ament_target_dependencies(control_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_include_directories(control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>)
rosidl_target_interfaces(control_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(test_node src/test_track_node.cpp)
ament_target_dependencies(test_node
  rclcpp
  geometry_msgs
  nav_msgs
)
target_include_directories(test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>)
rosidl_target_interfaces(test_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  control_node
  test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
