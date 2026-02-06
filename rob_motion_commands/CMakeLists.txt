cmake_minimum_required(VERSION 3.16)
project(rob_motion_commands)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dsl_intp REQUIRED)
find_package(scurve_lib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
#find_package(pybind11 CONFIG REQUIRED)
find_package(robnux_trajectory REQUIRED)
find_package(robnux_kdl_common REQUIRED)
find_package(simple_motion_logger REQUIRED)
find_package(robnux_kinematics_map REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pybind11 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${PYTHON3_INCLUDE_DIRS}
  ${robnux_trajectory_INCLUDE_DIRS}
  ${robnux_kdl_common_INCLUDE_DIRS}
  ${simple_motion_logger_INCLUDE_DIRS}
  ${robnux_kinematics_map_INCLUDE_DIRS}
  ${scurve_lib_INCLUDE_DIRS}
  ${dsl_intp_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
)

pybind11_add_module(${PROJECT_NAME} src/module_motion_command.cpp)

# Use plain signature for target_link_libraries (not keyword)
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${sensor_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${dsl_intp_LIBRARIES}
    ${scurve_lib_LIBRARIES}
    ${robnux_trajectory_LIBRARIES}
    ${robnux_kdl_common_LIBRARIES}
    ${simple_motion_logger_LIBRARIES}
    ${robnux_kinematics_map_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${rcl_interfaces_LIBRARIES}
    ${pluginlib_LIBRARIES}
    Eigen3::Eigen 
    ${PYTHON3_LIBRARIES}
)

  
#install(TARGETS ${PROJECT_NAME}
#     COMPONENT python3
#     LIBRARY DESTINATION "lib/python3/site-packages"
#)
ament_python_install_package(${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

ament_package()