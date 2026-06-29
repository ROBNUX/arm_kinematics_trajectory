cmake_minimum_required(VERSION 3.16)
project(rob_motion_commands)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav_msgs REQUIRED)
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
# Install the extension module inside the rob_motion_commands/ Python package
# so that `import rob_motion_commands` finds the package __init__.py (which
# re-exports everything from the .so via `from .rob_motion_commands import *`).
_ament_cmake_python_register_environment_hook()

install(TARGETS ${PROJECT_NAME}
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

install(FILES rob_motion_commands/__init__.py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

install(PROGRAMS
  examples/01_geometric_types.py
  examples/02_refpose_motion_types.py
  examples/03_scurve_planner.py
  examples/04_ros_lifecycle.py
  examples/05_forward_inverse_kinematics.py
  examples/06_cartesian_motion.py
  examples/07_relative_motion.py
  examples/08_joint_space_motion.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_geometric_types test/test_geometric_types.cpp)
  target_include_directories(test_geometric_types PRIVATE
    ${robnux_kdl_common_INCLUDE_DIRS}
    ${robnux_trajectory_INCLUDE_DIRS}
    ${robnux_kinematics_map_INCLUDE_DIRS}
    ${simple_motion_logger_INCLUDE_DIRS}
  )
  target_link_libraries(test_geometric_types
    robnux_kdl_common::robnux_kdl_common
    robnux_trajectory::robnux_trajectory
    simple_motion_logger::simple_motion_logger
    Eigen3::Eigen
  )

  ament_add_pytest_test(test_bindings test/test_bindings.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )
endif()

ament_package()
