cmake_minimum_required(VERSION 3.16)
project(robnux_kinematics_map)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(simple_motion_logger REQUIRED)
find_package(robnux_kdl_common REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include
${simple_motion_logger_INCLUDE_DIRS}
${robnux_kdl_common_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
${pluginlib_INCLUDE_DIRS}
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
            src/quattro.cpp
            src/quattroK.cpp
            src/quattro_4.cpp
            src/scara.cpp
            )
#target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} simple_motion_logger Eigen3 robnux_kdl_common pluginlib)
set_target_properties(${PROJECT_NAME} PROPERTIES DEFINE_SYMBOL KINEMATICS_EXPORTS)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES kinematics_plugin.xml
 DESTINATION share/${PROJECT_NAME}
)

ament_package()

