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

file(GLOB_RECURSE KINEMATICS_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_library(${PROJECT_NAME} SHARED ${KINEMATICS_SOURCES})

# Attach the include path to the target so the exported imported target
# (robnux_kinematics_map::robnux_kinematics_map) carries
# INTERFACE_INCLUDE_DIRECTORIES for downstream consumers.
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

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
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(simple_motion_logger robnux_kdl_common pluginlib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics_map test/test_kinematics_map.cpp)
  target_link_libraries(test_kinematics_map ${PROJECT_NAME})
endif()

ament_package()

