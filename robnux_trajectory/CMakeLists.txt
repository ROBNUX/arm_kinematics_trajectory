cmake_minimum_required(VERSION 3.16)
project(robnux_trajectory)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(simple_motion_logger REQUIRED)
find_package(robnux_kdl_common REQUIRED)
find_package(robnux_kinematics_map REQUIRED)
find_package(scurve_lib REQUIRED)

include_directories(include
${simple_motion_logger_INCLUDE_DIRS}
${robnux_kdl_common_INCLUDE_DIRS}
${robnux_kinematics_map_INCLUDE_DIRS}
${scurve_lib_INCLUDE_DIRS}
)

ament_export_include_directories(include)
if (WIN32)
  set(MYPREFIX_INCLUDE_DIRS "C:/robnux/include")
  set(MYPREFIX_LINK_DIRS "C:/robnux/lib")
  include_directories(${MYPREFIX_INCLUDE_DIRS})
  link_directories (${MYPREFIX_LINK_DIRS})
endif()

INSTALL(DIRECTORY include/
    DESTINATION include
)

add_library(${PROJECT_NAME} SHARED
            src/s_curve.cpp
            src/line_trajectory.cpp
            src/arc_trajectory.cpp
            src/shortest_quat_trajectory.cpp 
            src/motionlinemapper.cpp
            src/move_command.cpp 
            src/trajectory.cpp
            src/trajectory_segment.cpp
            src/PTPTrajectory.cpp
            src/trajectory_buffer.cpp
            src/base_profile.cpp
           )
ament_target_dependencies(${PROJECT_NAME} simple_motion_logger robnux_kdl_common robnux_kinematics_map scurve_lib)
set_target_properties(${PROJECT_NAME} PROPERTIES DEFINE_SYMBOL TRAJECTORY_EXPORTS)
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
  scurve_lib
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  ament_target_dependencies(test_trajectory
    simple_motion_logger robnux_kdl_common robnux_kinematics_map scurve_lib)
  target_link_libraries(test_trajectory ${PROJECT_NAME})
endif()

ament_package()
