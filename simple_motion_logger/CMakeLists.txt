cmake_minimum_required(VERSION 3.16)
project(simple_motion_logger)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)

include_directories(
    include
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
            src/Logger.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES DEFINE_SYMBOL LOGGER_EXPORTS)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

add_executable(testLogger src/main.cpp)
target_link_libraries(testLogger ${PROJECT_NAME})

install(TARGETS testLogger
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
