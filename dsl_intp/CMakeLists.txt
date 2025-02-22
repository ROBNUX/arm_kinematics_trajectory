cmake_minimum_required(VERSION 3.15...3.19)
project(dsl_intp)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    std_msgs
    simple_motion_logger
    kdl_common
    kinematics_map
    pluginlib
    trajectory
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dsl_intp
  CATKIN_DEPENDS
   roscpp
   sensor_msgs
   geometry_msgs
   std_msgs
   simple_motion_logger
   kdl_common
   kinematics_map
   pluginlib
   trajectory
)
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
else()
  find_package(Eigen3)
  if(NOT EIGEN3_FOUND)
    #  find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(Eigen3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  endif()
  include_directories(${EIGEN3_INCLUDE_DIRS})
endif()

add_library(${PROJECT_NAME} SHARED src/intp.cpp)

if (WIN32)
  TARGET_LINK_LIBRARIES(
     ${PROJECT_NAME}
     kdl_common
     trajectory
     kinematics_map
     simple_motion_logger
     pthreadVC2
     ${PYTHON_LIBRARIES}
  )
else()
  TARGET_LINK_LIBRARIES(
     ${PROJECT_NAME}
     trajectory
     pthread
     ${PYTHON_LIBRARIES}
    ${catkin_LIBRARIES}
  )
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES DEFINE_SYMBOL INTP_EXPORTS)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

INSTALL(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(
  testIntp
  test/testIntp.cpp
)

if (WIN32)
  TARGET_LINK_LIBRARIES(
     testIntp
     dsl_intp
     kdl_common
     trajectory
     kinematics_map
     simple_motion_logger
     pthreadVC2
  )
else()
  TARGET_LINK_LIBRARIES(
     testIntp
     dsl_intp
     ${PYTHON_LIBRARIES}
     ${catkin_LIBRARIES}
     pthread
  )
endif()

install(TARGETS testIntp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
