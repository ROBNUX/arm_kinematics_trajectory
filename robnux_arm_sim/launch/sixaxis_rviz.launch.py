#!/usr/bin/env python3
"""
Launch SixAxis_1 6-DOF arm in RViz2.

Usage:
    ros2 launch robnux_arm_sim sixaxis_rviz.launch.py
    ros2 launch robnux_arm_sim sixaxis_rviz.launch.py use_gui:=true
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_robot_description(xacro_path: str) -> str:
    result = subprocess.run(
        ["xacro", xacro_path], capture_output=True, text=True, check=True
    )
    return result.stdout


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("robnux_arm_sim")
    xacro_path = os.path.join(pkg, "urdf", "sixaxis_robot.urdf.xacro")
    rviz_cfg = os.path.join(pkg, "config", "arm_sim.rviz")

    robot_description = get_robot_description(xacro_path)
    use_gui = LaunchConfiguration("use_gui", default="false")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gui",
            default_value="false",
            description="Launch joint_state_publisher_gui for manual joint control",
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
            condition=UnlessCondition(use_gui),
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(use_gui),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg],
            output="screen",
        ),
    ])
