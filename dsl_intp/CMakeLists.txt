cmake_minimum_required(VERSION 3.16)
project(dsl_intp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(simple_motion_logger REQUIRED)
find_package(robnux_kdl_common REQUIRED)
find_package(robnux_kinematics_map REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robnux_trajectory REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python COMPONENTS Interpreter Development REQUIRED)

include_directories(include)
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

if (WIN32)
  target_link_libraries(
     ${PROJECT_NAME}
     robnux_kdl_common
     robnux_trajectory
     robnux_kinematics_map
     simple_motion_logger
     pthreadVC2
     ${PYTHON_LIBRARIES}
  )
else()
  target_link_libraries(
     ${PROJECT_NAME}
     robnux_trajectory
     pthread
     ${PYTHON_LIBRARIES}
  )
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES DEFINE_SYMBOL INTP_EXPORTS)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

add_executable(
  testIntp
  test/testIntp.cpp
)

if (WIN32)
  target_link_libraries(
     testIntp
     ${PROJECT_NAME}
     robnux_kdl_common
     robnux_trajectory
     robnux_kinematics_map
     simple_motion_logger
     pthreadVC2
  )
else()
  target_link_libraries(
     testIntp
     ${PROJECT_NAME}
     ${PYTHON_LIBRARIES}
     pthread
  )
endif()

install(TARGETS testIntp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
