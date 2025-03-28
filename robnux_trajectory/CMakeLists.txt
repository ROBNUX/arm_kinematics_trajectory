cmake_minimum_required(VERSION 3.15...3.19)
project(robnux_trajectory)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    simple_motion_logger
    robnux_kdl_common
    robnux_kinematics_map
    scurve_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory
  CATKIN_DEPENDS
   roscpp
   simple_motion_logger
   robnux_kdl_common
   robnux_kinematics_map
   scurve_lib
)

if (WIN32)
  set(MYPREFIX_INCLUDE_DIRS "C:/robnux/include")
  set(MYPREFIX_LINK_DIRS "C:/robnux/lib")
  include_directories(${MYPREFIX_INCLUDE_DIRS})
  link_directories (${MYPREFIX_LINK_DIRS})
endif()

INSTALL(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
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
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
set_target_properties(${PROJECT_NAME} PROPERTIES DEFINE_SYMBOL TRAJECTORY_EXPORTS)
INSTALL(TARGETS  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
