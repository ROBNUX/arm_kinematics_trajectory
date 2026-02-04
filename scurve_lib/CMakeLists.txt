cmake_minimum_required(VERSION 3.16)
project(scurve_lib)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(simple_motion_logger REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
   include
   ${simple_motion_logger_INCLUDE_DIRS}
   ${EIGEN3_INCLUDE_DIRS}
)

ament_export_include_directories(include)
if (WIN32)
  set(MYPREFIX_INCLUDE_DIRS "C:/robnux/include")
  set(MYPREFIX_LINK_DIRS "C:/robnux/lib")
  include_directories(${MYPREFIX_INCLUDE_DIRS})
  link_directories (${MYPREFIX_LINK_DIRS})
endif()

INSTALL(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

add_library(${PROJECT_NAME} SHARED
            src/computeRootsUtil.cpp
            src/piecewise_function.cpp
            src/segment_planning.cpp
            )
#target_link_libraries(${PROJECT_NAME} Eigen3::Eigen simple_motion_logger)
ament_target_dependencies(${PROJECT_NAME} simple_motion_logger Eigen3)
set_target_properties(${PROJECT_NAME} PROPERTIES DEFINE_SYMBOL SCURVE_EXPORTS)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(testCubic test/testCubicRoots.cpp)
#target_link_libraries(testCubic ${PROJECT_NAME} Eigen3::Eigen simple_motion_logger)
ament_target_dependencies(testCubic simple_motion_logger Eigen3)
target_link_libraries(testCubic ${PROJECT_NAME})

add_executable(testTrajSeg test/testTrajSegment.cpp)
#target_link_libraries(testTrajSeg ${PROJECT_NAME} Eigen3::Eigen simple_motion_logger)
ament_target_dependencies(testTrajSeg simple_motion_logger Eigen3)
target_link_libraries(testTrajSeg ${PROJECT_NAME})

add_executable(testTrajMidPoint test/testTrajSegmentMidPoints.cpp)
#target_link_libraries(testTrajMidPoint ${PROJECT_NAME} Eigen3::Eigen simple_motion_logger)
ament_target_dependencies(testTrajMidPoint simple_motion_logger Eigen3)
target_link_libraries(testTrajMidPoint ${PROJECT_NAME})

ament_package()