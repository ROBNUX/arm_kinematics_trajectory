cmake_minimum_required(VERSION 3.16)
project(quattro_bot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robnux_kinematics_map REQUIRED)
find_package(robnux_kdl_common REQUIRED)
find_package(robnux_trajectory REQUIRED)
find_package(scurve_lib REQUIRED)
find_package(simple_motion_logger REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  ${robnux_kinematics_map_INCLUDE_DIRS}
  ${robnux_kdl_common_INCLUDE_DIRS}
  ${robnux_trajectory_INCLUDE_DIRS}
  ${simple_motion_logger_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${scurve_lib_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

add_executable(
  test_quattro_rviz
  test/test_quattro_rviz.cpp
)

ament_target_dependencies(
  test_quattro_rviz
  sensor_msgs
  std_msgs
  geometry_msgs
  robnux_kinematics_map
  robnux_kdl_common
  robnux_trajectory
  scurve_lib
  simple_motion_logger
  rclcpp
)

install(TARGETS test_quattro_rviz
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(
  test_quattro4_rviz
  test/test_quattro4_rviz.cpp
)
ament_target_dependencies(
  test_quattro4_rviz
  sensor_msgs
  std_msgs
  geometry_msgs
  robnux_kinematics_map
  robnux_kdl_common
  robnux_trajectory
  scurve_lib
  simple_motion_logger
  rclcpp
)

install(TARGETS test_quattro4_rviz
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY yaml
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY model
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()