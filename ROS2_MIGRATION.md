# ROS1 to ROS2 Migration Guide
## arm_kinematics_trajectory Repository

### Overview
This document describes the migration of the `arm_kinematics_trajectory` repository from ROS1 to ROS2. The migration has been completed and includes updates to all configuration files, build system files, and launch files.

### Migration Summary

#### Completed Changes

##### 1. Package Configuration (package.xml)
- **Format**: Changed from format 2 to **format 3**
- **Build System**: Changed from `catkin` to **`ament_cmake`**
- **Dependency Tags**: Consolidated dependencies using:
  - `build_depend`: Packages needed at build time
  - `exec_depend`: Packages needed at runtime
  - Removed unused `roscpp` dependencies from most packages

**Packages Updated:**
- `dsl_intp`
- `quattro_bot`
- `robnux_kdl_common`
- `robnux_kinematics_map`
- `robnux_trajectory`
- `scurve_lib`
- `simple_motion_logger`
- `rob_motion_commands`

##### 2. Build Configuration (CMakeLists.txt)
All CMakeLists.txt files have been updated with the following changes:

**Key Changes:**
- `cmake_minimum_required(VERSION 3.16)` (was 3.15...3.19)
- `find_package(ament_cmake REQUIRED)` (was `find_package(catkin REQUIRED)`)
- `add_compile_options(-std=c++11)` → `set(CMAKE_CXX_STANDARD 11)` pattern
- `include_directories()` statements simplified (no more `${catkin_INCLUDE_DIRS}`)
- `catkin_package()` → `ament_package()` at end of file
- `TARGET_LINK_LIBRARIES()` → `target_link_libraries()` (lowercase)
- `INSTALL()` → `install()` (lowercase)
- Installation destinations changed:
  - `${CATKIN_PACKAGE_INCLUDE_DESTINATION}` → `include/${PROJECT_NAME}`
  - `${CATKIN_PACKAGE_LIB_DESTINATION}` → `lib`
  - `${CATKIN_PACKAGE_SHARE_DESTINATION}` → `share/${PROJECT_NAME}`

**Files Updated:**
- `simple_motion_logger/CMakeLists.txt`
- `scurve_lib/CMakeLists.txt`
- `robnux_kdl_common/CMakeLists.txt`
- `robnux_kinematics_map/CMakeLists.txt`
- `robnux_trajectory/CMakeLists.txt`
- `dsl_intp/CMakeLists.txt`
- `rob_motion_commands/CMakeLists.txt`
- `quattro_bot/CMakeLists.txt`

##### 3. Launch Files
All ROS1 XML launch files have been converted to ROS2 Python launch files:

**New Python Launch Files Created:**
- `quattro_bot/launch/quattro_world.launch.py` (from `quattro_world.launch`)
- `quattro_bot/launch/quattro_rviz.launch.py` (from `quattro_rviz.launch`)
- `quattro_bot/launch/quattro_4_rviz.launch.py` (from `quattro_4_rviz.launch`)
- `quattro_bot/launch/test_quattro.launch.py` (from `test_quattro.launch`)
- `quattro_bot/launch/test_quattro_rviz.launch.py` (from `test_quattro_rviz.launch`)

**Original XML Files:**
- Original `.launch` files are still present but deprecated. They can be removed after verification.

#### No Source Code Changes Required

The codebase has minimal ROS-specific API usage:
- **simple_motion_logger**: File-based logging, no ROS dependencies
- **scurve_lib**: Pure motion planning library
- **robnux_kdl_common**: Pure math/kinematics library
- **robnux_kinematics_map**: Plugin-based architecture (compatible with ROS2)
- **robnux_trajectory**: Trajectory planning, minimal ROS interaction
- **dsl_intp**: Interpreter library with pybind11 bindings
- **quattro_bot**: Simulation/testing nodes

No C++ source files required updates for ROS API changes.

### Building and Testing

#### Prerequisites
- ROS2 (Humble or later recommended)
- Python 3.8+
- cmake >= 3.16
- Eigen3
- pybind11

#### Build Instructions

```bash
# Source ROS2 setup
source /opt/ros/<distro>/setup.bash

# Navigate to workspace
cd ~/robnux

# Build the packages
colcon build --symlink-install

# Source the install space
source install/setup.bash
```

#### Running Tests

```bash
# Run specific package tests
colcon test --packages-select dsl_intp
colcon test --packages-select simple_motion_logger
colcon test --packages-select scurve_lib

# View test results
colcon test-result --all --verbose
```

#### Launching Simulations

```bash
# ROS2 launch syntax
ros2 launch quattro_bot quattro_rviz.launch.py
ros2 launch quattro_bot quattro_world.launch.py
ros2 launch quattro_bot test_quattro.launch.py
```

### Migration Notes

1. **Dependency Management**: ROS2 uses clearer separation between build and runtime dependencies. If you encounter missing dependencies at build or runtime, check the `package.xml` files.

2. **Plugin System**: The `kinematics_plugin.xml` in `robnux_kinematics_map` is still valid for ROS2 plugin discovery.

3. **Launch Files**: The Python-based launch files provide better type safety and IDE support compared to XML. They follow ROS2 launch conventions.

4. **Environment Variables**: No more `find` substitutions. Use Python `get_package_share_directory()` and path operations instead.

5. **Package Naming**: All package names remain the same for compatibility.

### Troubleshooting

#### Build Failures

1. **Missing Dependencies**: Ensure all system dependencies are installed:
   ```bash
   sudo apt install ros-<distro>-<package-name>
   ```

2. **Eigen3 Not Found**: Install Eigen3:
   ```bash
   sudo apt install libeigen3-dev
   ```

3. **pybind11 Not Found**: Install pybind11:
   ```bash
   sudo apt install pybind11-dev
   ```

#### Runtime Issues

1. **Library Not Found**: Check LD_LIBRARY_PATH:
   ```bash
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/robnux/install/lib
   ```

2. **Launch File Not Found**: Ensure proper sourcing:
   ```bash
   source ~/robnux/install/setup.bash
   ```

### Next Steps

1. **Testing**: Run full test suite on target ROS2 distribution
2. **Documentation**: Update README files with ROS2-specific instructions
3. **CI/CD**: Update build pipelines to use colcon and ROS2 tools
4. **Deprecation**: Consider removing old ROS1 XML launch files after verification
5. **Validation**: Test all simulation and motion planning workflows

### References

- [ROS2 Migration Guide](https://docs.ros.org/en/humble/Contributing/Migration-Guide.html)
- [ROS2 Package Format](https://docs.ros.org/en/humble/Concepts/Basic/About-ROS2-Packages.html)
- [ROS2 CMake Conventions](https://cmake.ros2.org/)
- [ROS2 Launch System](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html)

---

**Migration Completed**: February 2026
**Status**: Ready for Testing
