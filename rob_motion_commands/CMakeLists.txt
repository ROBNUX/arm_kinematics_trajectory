cmake_minimum_required(VERSION 3.16)
project(rob_motion_commands)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dsl_intp REQUIRED)
find_package(scurve_lib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python 3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

if (WIN32)
  set(EIGEN3_INCLUDE_DIRS "C:/eigen3/build/install/include")
  set(MYPREFIX_INCLUDE_DIRS "C:/robnux/include")
  set(MYPREFIX_LINK_DIRS "C:/robnux/lib")
  set(PTHREAD_INCLUDE_DIRS "C:/pthreads-w32-2-9-1-release/Pre-built.2/include")
  set(PTHREAD_LINK_DIRS "C:/pthreads-w32-2-9-1-release/Pre-built.2/lib/x64")
  include_directories(${PTHREAD_INCLUDE_DIRS})
  include_directories(${MYPREFIX_INCLUDE_DIRS})
  link_directories (${MYPREFIX_LINK_DIRS})
  link_directories(${PTHREAD_LINK_DIRS})
  
  pybind11_add_module(${PROJECT_NAME} src/intp/module_motion_command.cpp)
  target_link_libraries(${PROJECT_NAME}
    PRIVATE
     dsl_intp
     scurve_lib
     pthreadVC2
     ${Python_LIBRARIES}
  )
else()
  pybind11_add_module(${PROJECT_NAME} src/intp/module_motion_command.cpp)
  target_link_libraries(${PROJECT_NAME}
    PRIVATE
    scurve_lib
    dsl_intp
    ${Python_LIBRARIES})
  
  install(TARGETS ${PROJECT_NAME}
     COMPONENT python3
     LIBRARY DESTINATION "lib/python3/site-packages"
  )
endif()

ament_package()