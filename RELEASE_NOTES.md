# Release Notes — arm_kinematics_trajectory

## v2.0.0 — ROS 2 migration & Python-binding overhaul (2026-05)

This is a **major release** spanning the full ROS 1 → ROS 2 (Humble)
migration of the workspace, a rename of every package to the
`robnux_*` / `*_motion_commands` / `*_intp` convention, a substantial
expansion of the pybind11 surface, and the introduction of a unit-test
suite across the workspace.

### ⚠️ Breaking changes

#### Package renames

| Old | New |
|---|---|
| `Logger` | `simple_motion_logger` |
| `kdl_common` | `robnux_kdl_common` |
| `trajectory` | `robnux_trajectory` |
| `intp` | `dsl_intp` |
| `rob_commands` | `rob_motion_commands` |
| `quattro` | `quattro_bot` |

Header include paths and Python module names change accordingly. The
new layout is consistent with the ROS 2 package-name conventions and
matches what's installed under `install/<pkg>/`.

A new package — `robnux_kinematics_map` — was split out from the
trajectory code to host all FK/IK plugin classes (`Scara`, `SixAxis_1`,
`UJNT`, `XYZGantry`, `XYZ_UR`, `singleAxisModule`, `Quattro`,
`QuattroK`, `Quattro_4`).

#### Build system

- Migrated from `catkin` to `colcon` / `ament_cmake`.
- `package.xml` files moved to format 3.
- Every library now uses modern target-based CMake: `ament_export_targets(... HAS_LIBRARY_TARGET)` plus `target_include_directories(... PUBLIC $<BUILD_INTERFACE:...> $<INSTALL_INTERFACE:include>)` so downstream namespaced targets (`<pkg>::<pkg>`) carry their own include paths.
- `target_link_libraries` now uses the keyword (`PRIVATE`/`PUBLIC`) form throughout — incompatible with old plain-signature `ament_target_dependencies` mixing.

#### C++ API

- `BaseKinematicMap::JntToCart` / `CartToJnt` output parameters changed from `Pose*` / `VectorXd*` / `Twist*` (pointer) to `Pose&` / `VectorXd&` / `Twist&` (reference) across all six overloads. All downstream call sites in `robnux_trajectory` and `dsl_intp` were updated.
- `BaseKinematicMap::SetDefaultBaseOff` is now a single-argument overload (`const Eigen::VectorXd& baseoff`); the previous two-argument variant taking a sub-base offset has been removed.
- `BaseKinematicMap::GetJntNames()` now returns `std::vector<std::string>&` (reference) instead of a pointer.
- `BaseKinematicMap::CalcPassive` final parameter changed from `VectorXd*` to `VectorXd&`.
- Implemented previously-missing acceleration-level FK/IK overloads on `XYZ_UR` (`JntToCart` and `CartToJnt` with `(q, qdot, qddot, p, v, a)` signatures) — they currently return `-1` (not implemented) but link cleanly so consumers don't see undefined-reference errors at load time.

#### ROS context lifecycle

- The `dsl_intp::CreateRobot` constructor still creates a `rclcpp::Node` and publishers internally, but the **ROS 2 context is no longer initialized inside `CreateRobot`**. Calling `CreateRobot` from C++ now requires `rclcpp::init` to have been called by the host application first.
- The Python binding (`rob_motion_commands`) calls `rclcpp::init()` automatically at module import and registers an `atexit` handler that calls `rclcpp::shutdown()` after Python destructors fire — so `import rob_motion_commands` is sufficient to make `CreateRobot` work from Python with no further setup.

### ✨ New features

#### `rob_motion_commands` — expanded Python bindings

The pybind11 module now exposes the full geometric primitive layer in
addition to motion commands:

- **Geometric types** (from `robnux_kdl_common`): `Vec`, `Quaternion`, `Rotation`, `Frame`, `Pose`, `refPose`. All major constructors, accessors, Euler-angle round-trip, `Frame::Inverse`, `Frame::DH_Craig1989`, `Frame::DH`, `Frame::Identity`.
- **Motion data types** (unchanged behavior, now with `py::arg` names): `MotionType`, `IpoMode`, `FrameData`, `Profile`, `Percent`, `JntProfile`, `LocData`.
- **ROS lifecycle helpers**: `ros_init()`, `ros_shutdown()`, `ros_ok()`.
- **`Robot` (CreateRobot)** — kinematics output methods now return Python tuples instead of writing through `Eigen::Ref` output parameters:
  - `GetCartFromJnt(jnt, cart_size) → (ok, cart)`
  - `GetPoseFromJnt(jnt, pose_size) → (ok, pose)`
  - `GetJntFromPose(pose, dof) → (ok, jnt)`
  - `ForwardKin(jnt) → (ok, LocData)`
  - `InverseKin(pose, dof) → (ok, jnt)`

#### Examples directory

`rob_motion_commands/examples/` ships four runnable scripts that double
as documentation:

- `01_geometric_types.py` — Vec / Quaternion / Rotation / Frame end-to-end
- `02_refpose_motion_types.py` — refPose with non-default base/tool, all motion data types
- `03_scurve_planner.py` — `ScurvePlanner.FitScurveSegment` with vel/acc/jerk-limit verification and optional matplotlib plot
- `04_ros_lifecycle.py` — `ros_init`/`ros_shutdown`/`ros_ok` idempotency

See [`rob_motion_commands/examples/README.md`](rob_motion_commands/examples/README.md).

#### Kinematic-model coverage

The kinematics plugin set now includes (under `robnux_kinematics_map`):

- `Scara` — 4-DoF SCARA
- `SixAxis_1` — 6-DoF serial arm
- `singleAxisModule` — single-axis prismatic/revolute module
- `UJNT` — universal-joint module
- `XYZGantry` — 3-axis Cartesian gantry
- `XYZ_UR` — 5-DoF XYZ stage + UR-style wrist
- `Quattro`, `QuattroK`, `Quattro_4` — three Quattro parallel-robot variants

### 🧪 Tests

Each library package now ships a `BUILD_TESTING`-guarded gtest suite:

| Package | Cases |
|---|---|
| `simple_motion_logger` | 5 |
| `scurve_lib` | 6 |
| `robnux_kdl_common` | 12 |
| `robnux_kinematics_map` | 10 |
| `robnux_trajectory` | 10 |
| `dsl_intp` | 2 |
| `rob_motion_commands` | 13 C++ (geometric types) + 13 pytest (bindings) |

Run with:

```bash
colcon test --packages-select simple_motion_logger scurve_lib \
  robnux_kdl_common robnux_kinematics_map robnux_trajectory dsl_intp \
  rob_motion_commands
colcon test-result --verbose
```

### 🔧 Quality & maintenance

- Google C++ style formatting applied to the pybind11 source via
  `clang-format --style=Google`.
- Removed stale `sensor_msgs` / `geometry_msgs` / `std_msgs` /
  `rclcpp` / `rcl_interfaces` / `pluginlib` dependencies where the
  source no longer uses them.
- Modernized `rob_motion_commands` install path: the `.so` is now
  installed directly at `${PYTHON_INSTALL_DIR}` (previously placed
  inside a `<pkg>/` subdirectory whose empty `__init__.py` was
  shadowing the extension on import).

### 🐛 Notable fixes

- Fixed link error in `librobnux_kinematics_map.so` caused by missing
  implementations of two `XYZ_UR` virtual overrides (acceleration-level
  `JntToCart` / `CartToJnt`).
- Fixed CMake "keyword vs plain `target_link_libraries`" conflict that
  was failing the build of the pybind11 modules when combined with
  `ament_target_dependencies`.
- Fixed `pybind11_add_module` install layout so `import rob_motion_commands`
  resolves to the extension rather than an empty `__init__.py`.

### 📦 Compatibility

- ROS 2 Humble (Ubuntu 22.04). Other distros not tested in this cycle.
- Eigen3, pybind11, pluginlib are external dependencies.
- The companion repo [`arm_kinematics_calibration`](https://github.com/ROBNUX/arm_kinematics_calibration)
  depends on this release and is upgraded in lock-step.

### 🙏 Migration tips for downstream consumers

1. Update all `#include "kdl_common/*.hpp"` → `#include "robnux_kdl_common/*.hpp"` and the same for the other renamed packages.
2. Replace `from rob_commands import *` with `from rob_motion_commands import ...`.
3. If you call `CreateRobot` from C++, add `rclcpp::init(argc, argv)` before constructing it. From Python no change is needed.
4. Any direct calls to `BaseKinematicMap::JntToCart` / `CartToJnt` that pass pointers must be converted to references.
5. Drop any second argument passed to `SetDefaultBaseOff`.

---

## v1.0.0 — Initial ROS 1 release (2025-02)

First public release. ROS 1 / catkin. Included:

- 7-segment jerk-limited S-curve trajectory profiler.
- Continuous smooth multi-segment Cartesian trajectories.
- FK/IK for `scara`, `quattro`, `quattroK`, `quattro_4`.
- Pybind11 robot scripting layer (`rob_commands`) with the `Robot` /
  `MoveLine` / `MoveArc` / `MovePTP` API.
- Hooks for one-step-forward / one-step-backward program-line mapping.
