cmake_minimum_required(VERSION 3.8)
project(robnux_arm_sim)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF/xacro models
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install SDF worlds
install(DIRECTORY sdf/
  DESTINATION share/${PROJECT_NAME}/sdf
)

# Install RViz configs
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python simulation scripts as executables
install(PROGRAMS
  scripts/run_simulation.py
  scripts/simulate_scara.py
  scripts/simulate_sixaxis.py
  scripts/simulate_6020_scara.py
  scripts/simulate_rv2fr.py
  scripts/simulate_xyz_gantry.py
  scripts/simulate_xyz_ur.py
  scripts/simulate_quattro.py
  scripts/simulate_poe_ur5.py
  scripts/simulate_poe_iiwa.py
  scripts/simulate_poe_panda.py
  scripts/simulate_poe_kuka.py
  scripts/simulate_poe_lite6.py
  scripts/record_sim.py
  scripts/trajectory_recorder.py
  scripts/test_fk_ik_sixaxis.py
  scripts/verify_sixaxis_ik_math.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
