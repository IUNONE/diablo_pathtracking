cmake_minimum_required(VERSION 3.5)
project(diablo_pathtracking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(motion_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(control_node src/diablo_track_node.cpp)
ament_target_dependencies(control_node
  rclcpp
  geometry_msgs
  nav_msgs
  motion_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(test_node src/test_track_node.cpp)
ament_target_dependencies(test_node
  rclcpp
  geometry_msgs
  nav_msgs
)

install(TARGETS
  control_node
  test_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
