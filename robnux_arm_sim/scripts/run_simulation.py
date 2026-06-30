#!/usr/bin/env python3
"""
Top-level simulation runner for all robnux kinematics types.

Usage:
    python3 run_simulation.py --robot scara
    python3 run_simulation.py --robot sixaxis_1
    python3 run_simulation.py --robot xyz_gantry
    python3 run_simulation.py --robot xyz_ur
    python3 run_simulation.py --robot quattro
    python3 run_simulation.py --robot quattro_4
    python3 run_simulation.py --robot poe_ur5
    python3 run_simulation.py --robot poe_iiwa
    python3 run_simulation.py --robot poe_panda
    python3 run_simulation.py --list

Or via ros2 run (after building the package):
    ros2 run robnux_arm_sim run_simulation.py --robot scara

Steps for full RViz visualization:
  1. In terminal 1: ros2 launch robnux_arm_sim <robot>_rviz.launch.py
  2. In terminal 2: python3 run_simulation.py --robot <robot>

The simulation script publishes /joint_states; robot_state_publisher
broadcasts TF; RViz2 renders the animated robot model.

Supported robot types and their key features:
  scara      - 4-DoF SCARA (Mitsubishi convention), DH serial arm
  sixaxis_1  - 6-DoF articulated arm (Mitsubishi 6-axis convention)
  xyz_gantry - 3-DoF Cartesian gantry (XYZ prismatic)
  xyz_ur     - 5-DoF XYZ gantry + 2-DoF tilt/pan UJNT head
  quattro    - 3-DoF parallel robot (XYZ translation)
  quattro_4  - 4-DoF parallel robot (XYZ + yaw)
  poe_ur5    - 6-DoF UR5-like arm  (POE ThreeParallel)
  poe_iiwa   - 7-DoF iiwa-like arm (POE SevenRSRS)
  poe_panda  - 7-DoF Panda-like arm (POE SevenRJointLock)
"""

import argparse
import sys


SUPPORTED_ROBOTS = {
    "scara":       ("simulate_scara",      "simulate_scara.main"),
    "sixaxis_1":   ("simulate_sixaxis",    "simulate_sixaxis.main"),
    "xyz_gantry":  ("simulate_xyz_gantry", "simulate_xyz_gantry.main"),
    "xyz_ur":      ("simulate_xyz_ur",     "simulate_xyz_ur.main"),
    "quattro":     ("simulate_quattro",    None),   # needs --four flag dispatch
    "quattro_4":   ("simulate_quattro",    None),
    "poe_ur5":     ("simulate_poe_ur5",    "simulate_poe_ur5.main"),
    "poe_iiwa":    ("simulate_poe_iiwa",   "simulate_poe_iiwa.main"),
    "poe_panda":   ("simulate_poe_panda",  "simulate_poe_panda.main"),
}

LAUNCH_MAP = {
    "scara":       "ros2 launch robnux_arm_sim scara_rviz.launch.py",
    "sixaxis_1":   "ros2 launch robnux_arm_sim sixaxis_rviz.launch.py",
    "xyz_gantry":  "ros2 launch robnux_arm_sim xyz_gantry_rviz.launch.py",
    "xyz_ur":      "ros2 launch robnux_arm_sim xyz_ur_rviz.launch.py",
    "quattro":     "ros2 launch quattro_bot quattro_rviz.launch.py",
    "quattro_4":   "ros2 launch quattro_bot quattro_4_rviz.launch.py",
    "poe_ur5":     "ros2 launch robnux_arm_sim poe_ur5_rviz.launch.py",
    "poe_iiwa":    "ros2 launch robnux_arm_sim poe_iiwa_rviz.launch.py",
    "poe_panda":   "ros2 launch robnux_arm_sim poe_panda_rviz.launch.py",
}


def list_robots() -> None:
    print("\nSupported kinematics types:\n")
    rows = [
        ("scara",       "4-DoF SCARA",                "Mitsubishi convention (R-R-P-R)"),
        ("sixaxis_1",   "6-DoF articulated arm",      "Mitsubishi 6-axis (all revolute)"),
        ("xyz_gantry",  "3-DoF Cartesian gantry",     "XYZ prismatic (Z-Y-X axes)"),
        ("xyz_ur",      "5-DoF XYZ + tilt/pan",       "Gantry + UJNT 2R head"),
        ("quattro",     "3-DoF parallel robot",       "XYZ translation (delta-like)"),
        ("quattro_4",   "4-DoF parallel robot",       "XYZ + yaw (Adept Quattro)"),
        ("poe_ur5",     "6-DoF POE UR5-like arm",     "ThreeParallel topology"),
        ("poe_iiwa",    "7-DoF POE iiwa-like arm",    "SevenRSRS topology"),
        ("poe_panda",   "7-DoF POE Panda-like arm",   "SevenRJointLock topology"),
    ]
    for name, dof_str, desc in rows:
        launch = LAUNCH_MAP.get(name, "")
        print(f"  {name:<14}  {dof_str:<26}  {desc}")
        print(f"               RViz: {launch}\n")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run a robnux arm simulation against the kinematics library.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--robot", "-r",
        choices=list(SUPPORTED_ROBOTS.keys()),
        help="Which kinematics type to simulate",
    )
    parser.add_argument(
        "--list", "-l",
        action="store_true",
        help="Print supported robot types and exit",
    )

    args = parser.parse_args()

    if args.list or args.robot is None:
        list_robots()
        if args.robot is None and not args.list:
            parser.print_usage()
        return 0

    robot = args.robot
    launch_cmd = LAUNCH_MAP.get(robot, "")

    print(f"\n{'='*60}")
    print(f"Starting simulation for: {robot}")
    print(f"{'='*60}")
    print(f"\nTIP: Launch RViz in a separate terminal first:")
    print(f"     {launch_cmd}\n")

    if robot == "scara":
        import simulate_scara
        return simulate_scara.main()

    elif robot == "sixaxis_1":
        import simulate_sixaxis
        return simulate_sixaxis.main()

    elif robot == "xyz_gantry":
        import simulate_xyz_gantry
        return simulate_xyz_gantry.main()

    elif robot == "xyz_ur":
        import simulate_xyz_ur
        return simulate_xyz_ur.main()

    elif robot in ("quattro", "quattro_4"):
        import simulate_quattro
        return simulate_quattro.run_quattro(dof4=(robot == "quattro_4"))

    elif robot == "poe_ur5":
        import simulate_poe_ur5
        return simulate_poe_ur5.main()

    elif robot == "poe_iiwa":
        import simulate_poe_iiwa
        return simulate_poe_iiwa.main()

    elif robot == "poe_panda":
        import simulate_poe_panda
        return simulate_poe_panda.main()

    else:
        print(f"ERROR: unknown robot '{robot}'")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
