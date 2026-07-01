#!/usr/bin/env python3
"""
Record sixaxis simulation topics to a ROS2 bag for detailed analysis.

Run this in a SEPARATE terminal BEFORE (or during) simulate_sixaxis.py:
    ros2 run robnux_arm_sim record_sim.py

Or specify a custom output path:
    ros2 run robnux_arm_sim record_sim.py ~/my_bags/test1

After recording, inspect the bag:
    ros2 bag info   ~/sim_bags/sixaxis_<timestamp>
    ros2 bag play   ~/sim_bags/sixaxis_<timestamp>

Extract joint data to CSV (example):
    ros2 topic echo --no-arr /joint_states --csv > joints.csv   # during replay

Topics recorded:
    /joint_states   sensor_msgs/JointState   — joint angles at each timestep
    /cart_pose      geometry_msgs/Pose       — Cartesian EE pose
    /arm_path       nav_msgs/Path            — accumulated EE path (RViz display)
"""

import argparse
import datetime
import os
import signal
import subprocess
import sys

TOPICS = [
    "/joint_states",
    "/cart_pose",
    "/arm_path",
]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Record sixaxis simulation topics to a ROS2 bag.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "output",
        nargs="?",
        default=None,
        help="Output bag directory (default: ~/sim_bags/sixaxis_YYYYMMDD_HHMMSS)",
    )
    parser.add_argument(
        "--all-topics",
        action="store_true",
        help="Record ALL active topics (includes /tf, /tf_static, etc.)",
    )
    args = parser.parse_args()

    if args.output is None:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_dir = os.path.expanduser(f"~/sim_bags/sixaxis_{ts}")
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
