cmake_minimum_required(VERSION 3.16)
project(quattro_bot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robnux_kinematics_map REQUIRED)
find_package(robnux_kdl_common REQUIRED)
find_package(robnux_trajectory REQUIRED)
find_package(simple_motion_logger REQUIRED)

add_executable(
  test_quattro_rviz
  test/test_quattro_rviz.cpp
)

target_link_libraries(
   test_quattro_rviz
   robnux_kinematics_map
   robnux_kdl_common
   robnux_trajectory
   simple_motion_logger
)

install(TARGETS test_quattro_rviz
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(
  test_quattro4_rviz
  test/test_quattro4_rviz.cpp
)

target_link_libraries(
   test_quattro4_rviz
   robnux_kinematics_map
   robnux_kdl_common
   robnux_trajectory
   simple_motion_logger
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