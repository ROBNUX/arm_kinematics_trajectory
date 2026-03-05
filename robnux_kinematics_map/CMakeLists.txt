cmake_minimum_required(VERSION 3.16)
project(robnux_kinematics_map VERSION 1.0.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(simple_motion_logger REQUIRED)
find_package(robnux_kdl_common REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${simple_motion_logger_INCLUDE_DIRS}
  ${robnux_kdl_common_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB_RECURSE KINEMATICS_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_library(${PROJECT_NAME} SHARED ${KINEMATICS_SOURCES})


target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}
  simple_motion_logger
  robnux_kdl_common
  pluginlib
)



# export macro for windows/shared builds
set_target_properties(${PROJECT_NAME} PROPERTIES DEFINE_SYMBOL KINEMATICS_EXPORTS)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES kinematics_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(simple_motion_logger robnux_kdl_common pluginlib)

ament_package()

