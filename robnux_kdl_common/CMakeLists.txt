cmake_minimum_required(VERSION 3.16)
project(robnux_kdl_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(simple_motion_logger REQUIRED)

include_directories(
  include
  ${simple_motion_logger_INCLUDE_DIRS} 
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
            src/pose.cpp
            src/rotation.cpp
            src/twistWrench.cpp
            src/vec.cpp)

ament_target_dependencies(${PROJECT_NAME} simple_motion_logger)
set_target_properties(${PROJECT_NAME} PROPERTIES DEFINE_SYMBOL COMMON_EXPORTS)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  simple_motion_logger
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()



