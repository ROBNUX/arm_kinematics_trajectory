# rob_motion_commands — Python examples

Runnable scripts that exercise the Python bindings exposed by
`rob_motion_commands`. They double as documentation and as smoke tests
beyond the formal pytest suite.

## Setup

```bash
cd ~/robnux/my_ws
source install/setup.bash
```

## Examples

| Script | What it shows |
|---|---|
| [01_geometric_types.py](01_geometric_types.py) | `Vec` / `Quaternion` / `Rotation` / `Frame` construction, accessors, and Euler-angle round-trip |
| [02_refpose_motion_types.py](02_refpose_motion_types.py) | `Pose` / `refPose` with non-default base & tool frames; `LocData`, `FrameData`, `Profile`, `JntProfile`, `Percent` field round-trip |
| [03_scurve_planner.py](03_scurve_planner.py) | `ScurvePlanner.FitScurveSegment` — fit a jerk-limited S-curve segment, dump the boundaries and (if matplotlib is installed) plot pos/vel/acc/jerk |
| [04_ros_lifecycle.py](04_ros_lifecycle.py) | `ros_init` / `ros_shutdown` / `ros_ok` — show that the ROS 2 context lifecycle is tied to the Python module and that the helpers are idempotent |

Run any of them with:

```bash
python3 src/arm_kinematics_trajectory/rob_motion_commands/examples/01_geometric_types.py
```

## Why no `CreateRobot` end-to-end example?

`CreateRobot` requires a real registered kinematics plugin (`scara`,
`quattro`, ...), DoF-shaped DH / base / tool vectors, and starts two
threads. A clean end-to-end example would also need a running rviz or
gazebo to make the publishers meaningful. Use [test_robot_language_script]
or the rviz launches in `quattro_bot` for that flow.
