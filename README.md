# arm_kinematics_trajectory

A ROS 2 workspace of C++ libraries (with Python bindings) for robotic-arm
kinematics, trajectory planning, and motion control. Includes a 7-segment
jerk-limited S-curve profiler, smooth multi-segment trajectories, FK/IK for
several robot families, and a small Python scripting layer for writing
robot programs.

![Smooth multi-segment trajectory](https://github.com/user-attachments/assets/b2073038-814c-4f3c-a8f6-55845c33fbd1)

## Packages

| Package | Purpose |
|---|---|
| [`simple_motion_logger`](simple_motion_logger/) | Thread-safe logger singleton used across the workspace |
| [`scurve_lib`](scurve_lib/) | Polynomial pieces + jerk-limited segment planning utilities |
| [`robnux_kdl_common`](robnux_kdl_common/) | `Vec`, `Quaternion`, `Rotation`, `Frame`, `Pose`, `refPose`, `Twist`, `Wrench` — foundational geometry types |
| [`robnux_kinematics_map`](robnux_kinematics_map/) | FK/IK plugin classes for Scara, Six-axis, Single-axis, UJNT, XYZ gantry, XYZ+UR, and three Quattro variants |
| [`robnux_trajectory`](robnux_trajectory/) | `SCurveProfile`, line/arc Cartesian trajectories, joint-space PTP, trajectory buffer, command queue |
| [`dsl_intp`](dsl_intp/) | `CreateRobot` interpreter — accepts robot-script commands and produces a trajectory buffer; publishes to ROS 2 topics for rviz/gazebo |
| [`rob_motion_commands`](rob_motion_commands/) | pybind11 module exposing all the above to Python |
| [`quattro_bot`](quattro_bot/) | Launch files, rviz configs, and meshes for the Quattro parallel robot |
| [`test_robot_language_script`](test_robot_language_script/) | Demo Python robot programs |

## Dependencies

ROS 2 (Humble or newer), Eigen3, pybind11, pluginlib, `rclcpp`,
`sensor_msgs` / `geometry_msgs` / `std_msgs`. Tests additionally need
`ament_cmake_gtest`.

## Build

```bash
cd ~/your_ws/src
git clone <this-repo>          # provides arm_kinematics_trajectory
cd ..
colcon build
source install/setup.bash
```

## Run the tests

```bash
colcon test --packages-select \
  simple_motion_logger scurve_lib robnux_kdl_common \
  robnux_kinematics_map robnux_trajectory dsl_intp rob_motion_commands
colcon test-result --verbose
```

Each library package ships a focused C++ gtest suite under `test/`.
`rob_motion_commands` additionally has a pytest suite exercising the
bindings end-to-end.

## Python quickstart

```python
import math
import numpy as np
from rob_motion_commands import (
    Robot, Profile, JntProfile, LocData, FrameData,
    IpoMode, WORLD,
)

# Cartesian profile (max_vel_t, max_acc_t, max_jerk_t,
#                    max_vel_r, max_acc_r, max_jerk_r)
pf = Profile(2, 50, 1500, 2, 50, 1500)

# Joint-space profile (per-joint vel/acc/jerk caps)
jpf = JntProfile(20, 50, 200)

# Kinematic parameters for the chosen plugin — see each robot's header
# for the parameter layout.
para = np.array([0.3, -math.pi / 2, 0.4, 0, 1.02, 0.1, 0,
                 0.18 * math.sqrt(2)]).reshape(-1, 1)

# Where the robot base sits in world coordinates (x, y, z, qw, qx, qy, qz).
default_base = np.array([0, 0, 0, 1, 0, 0, 0]).reshape(-1, 1)

robot = Robot("quattro", para, default_base, default_base, pf)

target = LocData(0.4, 0.4, -0.75, 0, 0, 0, branch=0, turns=0)
frame = FrameData(base=1, tool=1, ipo=IpoMode.WORLD)
robot.MoveLine(target, frame, 10)   # straight-line move, 10 = blending %
```

More end-to-end examples live in [`rob_motion_commands/examples/`](rob_motion_commands/examples/)
and [`test_robot_language_script/`](test_robot_language_script/).

## Primitive motion commands

Beyond the `Robot.Move*` API, the underlying C++ trajectory primitives are
also available: `LIN`, `PTP`, `ARC`, `LIN_REL`, `PTP_REL`, plus joint-space
`PTPJ`. The trajectory buffer leaves hooks for one-step-forward /
one-step-backward debugging of robot programs.

## ROS 2 lifecycle

`Robot` (a.k.a. `CreateRobot`) creates a `rclcpp::Node` and publishers in
its constructor for rviz/gazebo visualization. The pybind module
initializes the ROS context on import and registers an `atexit` handler
that calls `rclcpp::shutdown()` after Python destructors run — so simply
importing `rob_motion_commands` is enough for the published topics to
work, and clean teardown happens automatically at interpreter exit.
