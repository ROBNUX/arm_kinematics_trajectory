cmake_minimum_required(VERSION 2.8.3)
project(quattro)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    sensor_msgs
    std_msgs
    kinematics_map
    kdl_common
    trajectory
    Logger
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}
  test/test_quattro_ros.cpp
)


TARGET_LINK_LIBRARIES(
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(
  test_quattro_rviz
  test/test_quattro_rviz.cpp
)


TARGET_LINK_LIBRARIES(
   test_quattro_rviz
   ${catkin_LIBRARIES}
)

install(TARGETS test_quattro_rviz
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(
  test_quattro4_rviz
  test/test_quattro4_rviz.cpp
)


TARGET_LINK_LIBRARIES(
   test_quattro4_rviz
   ${catkin_LIBRARIES}
)

install(TARGETS test_quattro4_rviz
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY model
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})