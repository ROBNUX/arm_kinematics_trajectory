cmake_minimum_required(VERSION 3.16)
project(dsl_intp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(simple_motion_logger REQUIRED)
find_package(robnux_kdl_common REQUIRED)
find_package(robnux_kinematics_map REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robnux_trajectory REQUIRED)
find_package(scurve_lib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python COMPONENTS Interpreter Development REQUIRED)

include_directories(include
${simple_motion_logger_INCLUDE_DIRS}
${robnux_kdl_common_INCLUDE_DIRS}
${robnux_kinematics_map_INCLUDE_DIRS}
${robnux_trajectory_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
${pluginlib_INCLUDE_DIRS}
${scurve_lib_INCLUDE_DIRS}
${PYTHON_INCLUDE_DIRS}
${sensor_msgs_INCLUDE_DIRS}
${geometry_msgs_INCLUDE_DIRS}
${std_msgs_INCLUDE_DIRS}
${rclcpp_INCLUDE_DIRS}
)
ament_export_include_directories(include)
if (WIN32)
  set(EIGEN3_INCLUDE_DIRS "C:/eigen3/build/install/include")
  set(MYPREFIX_INCLUDE_DIRS "C:/robnux/include")
  set(MYPREFIX_LINK_DIRS "C:/robnux/lib")
  set(PTHREAD_INCLUDE_DIRS "C:/pthreads-w32-2-9-1-release/Pre-built.2/include")
  set(PTHREAD_LINK_DIRS "C:/pthreads-w32-2-9-1-release/Pre-built.2/lib/x64")
  include_directories(${PTHREAD_INCLUDE_DIRS})
  include_directories(${EIGEN3_INCLUDE_DIRS})
  include_directories(${MYPREFIX_INCLUDE_DIRS})
  link_directories (${MYPREFIX_LINK_DIRS})
  link_directories(${PTHREAD_LINK_DIRS})
endif()

add_library(${PROJECT_NAME} SHARED src/intp.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  simple_motion_logger
  robnux_kdl_common
  robnux_kinematics_map
  robnux_trajectory
  Eigen3
  pluginlib
  scurve_lib
)
set_target_properties(${PROJECT_NAME} PROPERTIES DEFINE_SYMBOL INTP_EXPORTS)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  simple_motion_logger
  robnux_kdl_common
  robnux_kinematics_map
  robnux_trajectory
  Eigen3
  pluginlib
  scurve_lib
  rclcpp
)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(DIRECTORY include/
    DESTINATION include
)

add_executable(
  testIntp
  test/testIntp.cpp
)

ament_target_dependencies(
  testIntp
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  simple_motion_logger
  robnux_kdl_common
  robnux_kinematics_map
  robnux_trajectory
  Eigen3
  pluginlib
  scurve_lib
)
target_link_libraries(
  testIntp
  ${PROJECT_NAME}
  pthread
  ${PYTHON_LIBRARIES}
)

install(TARGETS testIntp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_intp test/test_intp.cpp)
  ament_target_dependencies(test_intp
    rclcpp simple_motion_logger robnux_kdl_common robnux_kinematics_map
    robnux_trajectory pluginlib scurve_lib Eigen3
    sensor_msgs geometry_msgs std_msgs)
  target_link_libraries(test_intp ${PROJECT_NAME})
endif()

ament_package()
