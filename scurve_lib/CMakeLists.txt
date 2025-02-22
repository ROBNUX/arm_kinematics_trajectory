cmake_minimum_required(VERSION 3.15...3.19)
project(scurve_lib)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    simple_motion_logger
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(Eigen3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scurve_lib
  CATKIN_DEPENDS
   roscpp
   simple_motion_logger
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
            src/computeRootsUtil.cpp
            src/piecewise_function.cpp
            src/segment_planning.cpp
            )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
set_target_properties(${PROJECT_NAME} PROPERTIES DEFINE_SYMBOL SCURVE_EXPORTS)
INSTALL(TARGETS  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(testCubic test/testCubicRoots.cpp)
TARGET_LINK_LIBRARIES(testCubic ${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(testTrajSeg test/testTrajSegment.cpp)
TARGET_LINK_LIBRARIES(testTrajSeg ${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(testTrajMidPoint test/testTrajSegmentMidPoints.cpp)
TARGET_LINK_LIBRARIES(testTrajMidPoint ${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)