cmake_minimum_required(VERSION 3.16)
project(scurve_lib)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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

## Install headers to the workspace include/ so ament consumers can find them
install(DIRECTORY include/
    DESTINATION include
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
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  Eigen3
  simple_motion_logger
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scurve test/test_scurve.cpp)
  ament_target_dependencies(test_scurve simple_motion_logger Eigen3)
  target_link_libraries(test_scurve ${PROJECT_NAME})
endif()

ament_package()