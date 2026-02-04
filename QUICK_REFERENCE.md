# ROS2 Migration Quick Reference

## What Changed

### Package Files (package.xml)
- **Format**: Now format="3"
- **Build tool**: catkin → ament_cmake
- **Dependencies**: Separated into build_depend and exec_depend

### Build Files (CMakeLists.txt)
- **Minimum CMake**: 3.15...3.19 → 3.16
- **Package manager**: catkin → ament_cmake
- **Package config**: catkin_package() → ament_package()
- **Link libraries**: TARGET_LINK_LIBRARIES → target_link_libraries
- **Install paths**: ${CATKIN_*} → standard lib/include/share paths

### Launch Files
- **Format**: XML → Python3
- **Syntax**: $(find pkg) → get_package_share_directory('pkg')
- **Nodes**: node type/pkg → package/executable

---

## Quick Build Commands

```bash
# Build all packages
colcon build --symlink-install

# Build specific package
colcon build --packages-select quattro_bot

# Build and test
colcon build --tests
colcon test

# Source setup
source install/setup.bash
```

---

## Quick Launch Commands

```bash
# ROS2 launch syntax
ros2 launch quattro_bot quattro_rviz.launch.py
ros2 launch quattro_bot quattro_world.launch.py
ros2 launch quattro_bot test_quattro.launch.py
ros2 launch quattro_bot quattro_4_rviz.launch.py
ros2 launch quattro_bot test_quattro_rviz.launch.py
```

---

## Packages Summary

| Package | Type | Purpose | Dependencies |
|---------|------|---------|--------------|
| simple_motion_logger | Library | Logging utilities | None (header-only) |
| scurve_lib | Library | S-curve trajectory | Eigen3 |
| robnux_kdl_common | Library | Kinematics common | None |
| robnux_kinematics_map | Library | FK/IK solver | pluginlib, Eigen3 |
| robnux_trajectory | Library | Motion planning | robnux_kdl_common |
| dsl_intp | Library | Interpreter API | Python, pybind11 |
| quattro_bot | Package | Simulation | All above + RViz |
| rob_motion_commands | Module | Python bindings | pybind11 |

---

## Troubleshooting

**Issue**: `Package 'X' not found`
```bash
# Make sure to source setup
source install/setup.bash
# Or rebuild
colcon build --packages-select X
```

**Issue**: `ament_cmake not found`
```bash
# Install ROS2 development tools
sudo apt install ros-<distro>-ament-cmake
sudo apt install ros-<distro>-ament-lint
```

**Issue**: Missing messages or dependencies
```bash
# Install ROS2 common packages
sudo apt install ros-<distro>-common-interfaces
```

---

## File Locations

```
arm_kinematics_trajectory/
├── */package.xml          # ROS2 package format 3
├── */CMakeLists.txt       # Ament CMake files
├── quattro_bot/
│   └── launch/
│       ├── *.launch       # Old (deprecated)
│       └── *.launch.py    # New ROS2 format
├── ROS2_MIGRATION.md      # Detailed guide
└── MIGRATION_SUMMARY.md   # This summary
```

---

## Environment Setup

```bash
# Set ROS_DISTRO (if not already set)
export ROS_DISTRO=humble  # or iron, rolling, etc.

# Source ROS2 setup
source /opt/ros/$ROS_DISTRO/setup.bash

# Setup workspace
cd ~/robnux
source install/setup.bash

# Verify setup
ros2 --version
```

---

## Common Issues & Solutions

| Error | Cause | Solution |
|-------|-------|----------|
| `colcon: command not found` | Missing colcon | `pip install colcon-common-extensions` |
| `find_package(ament_cmake)` failed | ament_cmake missing | `sudo apt install ros-$ROS_DISTRO-ament-cmake` |
| Packages not found after build | Forgot to source | `source install/setup.bash` |
| Old catkin commands fail | Using ROS1 tools | Use colcon instead |
| Launch file not found | Path issues | Use full path or proper package name |

---

## Documentation Files

- **ROS2_MIGRATION.md**: Complete migration guide with all details
- **MIGRATION_SUMMARY.md**: This comprehensive summary report
- **README.md** (original): Project overview (may need updates)

---

## Key Differences ROS1 → ROS2

```
ROS1                           ROS2
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
catkin_make                    colcon build
rosbuild                       ament_cmake
package.xml format 2           package.xml format 3
*.launch (XML)                 *.launch.py (Python)
roslaunch pkg file.launch      ros2 launch pkg file.launch.py
ROS Parameter Server           Node parameters
rosgraph                        ros2 graph
roscpp                          removed (not needed)
```

---

## Version Information

- **CMake**: >= 3.16 (was 3.15...3.19)
- **C++ Standard**: C++11 (unchanged)
- **Python**: 3.8+ (for bindings)
- **Build System**: ament_cmake
- **ROS2 Version**: Humble or later recommended

---

**Status**: ✅ Migration Complete & Ready for Use

For detailed information, see:
- `ROS2_MIGRATION.md` - Full migration guide
- `MIGRATION_SUMMARY.md` - Complete summary report
