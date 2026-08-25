#!/usr/bin/env python3
"""
Record any robnux_arm_sim simulation's topics to a ROS2 bag for detailed
analysis or replay -- works with any simulate_*.py, since they all publish
the same topic names.

Run this in a SEPARATE terminal BEFORE (or during) any simulate_*.py:
    ros2 run robnux_arm_sim record_sim.py

Or specify a custom output path:
    ros2 run robnux_arm_sim record_sim.py ~/my_bags/test1

Also start trajectory_recorder.py first (in a third terminal, or via a
*_rviz.launch.py record:=true launch arg) to get joint/Cartesian velocity
and acceleration in the bag -- /joint_states and /cart_pose only carry
position on their own:
    ros2 run robnux_arm_sim trajectory_recorder.py

After recording, inspect the bag:
    ros2 bag info   ~/sim_bags/sim_<timestamp>
    ros2 bag play   ~/sim_bags/sim_<timestamp>

Extract joint data to CSV (example):
    ros2 topic echo --no-arr /joint_states_full --csv > joints.csv   # during replay

Topics recorded:
    /joint_states        sensor_msgs/JointState      — joint angles (position only)
    /joint_states_full   sensor_msgs/JointState      — position+velocity+accel(effort),
                                                         from trajectory_recorder.py if running
    /cart_pose           geometry_msgs/PoseStamped   — Cartesian EE pose
    /cart_twist          geometry_msgs/TwistStamped  — Cartesian EE velocity, from
                                                         trajectory_recorder.py if running
    /cart_accel          geometry_msgs/AccelStamped  — Cartesian EE acceleration, from
                                                         trajectory_recorder.py if running
    /arm_path            nav_msgs/Path               — accumulated EE path (RViz display)
"""

import argparse
import datetime
import os
import signal
import subprocess
import sys

TOPICS = [
    "/joint_states",
    "/joint_states_full",
    "/cart_pose",
    "/cart_twist",
    "/cart_accel",
    "/arm_path",
]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Record robnux_arm_sim simulation topics to a ROS2 bag.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "output",
        nargs="?",
        default=None,
        help="Output bag directory (default: ~/sim_bags/sim_YYYYMMDD_HHMMSS)",
    )
    parser.add_argument(
        "--all-topics",
        action="store_true",
        help="Record ALL active topics (includes /tf, /tf_static, etc.)",
    )
    args = parser.parse_args()

    if args.output is None:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_dir = os.path.expanduser(f"~/sim_bags/sim_{ts}")
    else:
        bag_dir = os.path.expanduser(args.output)

    os.makedirs(os.path.expanduser("~/sim_bags"), exist_ok=True)

    if args.all_topics:
        cmd = ["ros2", "bag", "record", "-o", bag_dir, "-a"]
        print(f"Recording ALL topics to: {bag_dir}")
    else:
        cmd = ["ros2", "bag", "record", "-o", bag_dir] + TOPICS
        print(f"Recording to:  {bag_dir}")
        print(f"Topics:        {', '.join(TOPICS)}")

    print("Press Ctrl+C to stop recording.\n")

    proc = subprocess.Popen(cmd)
    try:
        proc.wait()
    except KeyboardInterrupt:
        proc.send_signal(signal.SIGINT)
        proc.wait()

    print(f"\nBag saved to: {bag_dir}")
    print(f"Info:   ros2 bag info {bag_dir}")
    print(f"Replay: ros2 bag play {bag_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
