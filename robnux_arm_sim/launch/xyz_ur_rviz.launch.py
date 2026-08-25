#!/usr/bin/env python3
"""
Launch XYZ_UR 5-DOF robot (XYZ gantry + UJNT tilt/pan) in RViz2.

Usage:
    ros2 launch robnux_arm_sim xyz_ur_rviz.launch.py
    ros2 launch robnux_arm_sim xyz_ur_rviz.launch.py use_gui:=true
    ros2 launch robnux_arm_sim xyz_ur_rviz.launch.py use_sim:=true
    ros2 launch robnux_arm_sim xyz_ur_rviz.launch.py use_sim:=true record:=true
"""

import datetime
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution, OrSubstitution
from launch_ros.actions import Node

# Recorded verbatim (position-only, as published): the raw ground truth.
# /joint_states_full, /cart_twist, /cart_accel are derived by
# trajectory_recorder.py (finite-differenced velocity/acceleration) and are
# only non-empty while that node is running, which record:=true also starts.
BAG_TOPICS = [
    "/joint_states", "/joint_states_full",
    "/cart_pose", "/cart_twist", "/cart_accel",
    "/arm_path",
]


def get_robot_description(xacro_path: str) -> str:
    result = subprocess.run(
        ["xacro", xacro_path], capture_output=True, text=True, check=True
    )
    return result.stdout


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("robnux_arm_sim")
    xacro_path = os.path.join(pkg, "urdf", "xyz_ur_robot.urdf.xacro")
    rviz_cfg = os.path.join(pkg, "config", "arm_sim.rviz")

    robot_description = get_robot_description(xacro_path)
    use_gui = LaunchConfiguration("use_gui", default="false")
    use_sim = LaunchConfiguration("use_sim", default="false")
    record = LaunchConfiguration("record", default="false")
    bag_path = LaunchConfiguration("bag_path")
    default_bag_path = os.path.expanduser(
        f"~/sim_bags/xyz_ur_{datetime.datetime.now():%Y%m%d_%H%M%S}"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gui",
            default_value="false",
            description="Launch joint_state_publisher_gui for manual joint control",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Suppress joint_state_publisher when an external simulation provides /joint_states",
        ),
        DeclareLaunchArgument(
            "record",
            default_value="false",
            description="Start trajectory_recorder.py (derives joint/Cartesian "
                        "velocity+acceleration) and ros2 bag record on the joint/Cartesian "
                        "trajectory topics (pos, vel, acc), for later replay/debugging",
        ),
        DeclareLaunchArgument(
            "bag_path",
            default_value=default_bag_path,
            description="Output directory for the rosbag (only used when record:=true)",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{"use_gui": False}],
            condition=UnlessCondition(OrSubstitution(use_gui, use_sim)),
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(AndSubstitution(use_gui, NotSubstitution(use_sim))),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg],
            output="screen",
        ),

        # Run the matching simulation script when use_sim:=true
        ExecuteProcess(
            cmd=["ros2", "run", "robnux_arm_sim", "simulate_xyz_ur.py"],
            output="screen",
            condition=IfCondition(use_sim),
        ),

        # Derives joint/Cartesian velocity+acceleration by differencing the
        # position-only /joint_states and /cart_pose topics (see
        # trajectory_recorder.py's module docstring for why differencing
        # rather than exposing the trajectory engine's internal state).
        Node(
            package="robnux_arm_sim",
            executable="trajectory_recorder.py",
            name="trajectory_recorder",
            output="screen",
            condition=IfCondition(record),
        ),

        ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", bag_path, "--topics"] + BAG_TOPICS,
            output="screen",
            condition=IfCondition(record),
        ),
    ])
