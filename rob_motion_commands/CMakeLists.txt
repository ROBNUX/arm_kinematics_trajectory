cmake_minimum_required(VERSION 3.16)
project(rob_motion_commands)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(dsl_intp REQUIRED)
find_package(scurve_lib REQUIRED)
find_package(robnux_trajectory REQUIRED)
find_package(robnux_kdl_common REQUIRED)
find_package(robnux_kinematics_map REQUIRED)
find_package(simple_motion_logger REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

pybind11_add_module(${PROJECT_NAME} src/module_motion_command.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
  dsl_intp::dsl_intp
  scurve_lib::scurve_lib
  robnux_trajectory::robnux_trajectory
  robnux_kdl_common::robnux_kdl_common
  simple_motion_logger::simple_motion_logger
  ${robnux_kinematics_map_LIBRARIES}
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  Eigen3::Eigen
  ${PYTHON3_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${dsl_intp_INCLUDE_DIRS}
  ${scurve_lib_INCLUDE_DIRS}
  ${robnux_trajectory_INCLUDE_DIRS}
  ${robnux_kdl_common_INCLUDE_DIRS}
  ${robnux_kinematics_map_INCLUDE_DIRS}
  ${simple_motion_logger_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
ament_python_install_package(${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

ament_package()
