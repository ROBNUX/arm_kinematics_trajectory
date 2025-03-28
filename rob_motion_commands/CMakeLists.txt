cmake_minimum_required(VERSION 3.15...3.19)
project(rob_motion_commands)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    std_msgs
    dsl_intp
    scurve_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
  CATKIN_DEPENDS
   roscpp
   sensor_msgs
   geometry_msgs
   std_msgs
   dsl_intp
   scurve_lib
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
set(Python_ADDITIONAL_VERSIONS 3.8)
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
if (WIN32)
  pybind11_add_module(${PROJECT_NAME} src/intp/module_motion_command.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  PRIVATE
   dsl_intp
   kdl_common
   trajectory
   kinematics_map
   pthreadVC2
   ${PYTHON_LIBRARIES}
)
else()
  catkin_python_setup()
  pybind11_add_module(${PROJECT_NAME} src/intp/module_motion_command.cpp)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})  
  TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    PRIVATE
    scurve_lib
    dsl_intp
    ${catkin_LIBRARIES})
  install(TARGETS ${PROJECT_NAME}
     COMPONENT python3
     LIBRARY DESTINATION "/opt/robnux/lib/python3/site-packages"
  )
endif()