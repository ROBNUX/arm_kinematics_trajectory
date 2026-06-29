#!/usr/bin/env python3
"""
Launch SCARA robot in RViz2.

Usage (from a sourced ROS2 workspace):
    ros2 launch robnux_arm_sim scara_rviz.launch.py
    ros2 launch robnux_arm_sim scara_rviz.launch.py use_gui:=true
    ros2 launch robnux_arm_sim scara_rviz.launch.py use_sim:=true
    ros2 launch robnux_arm_sim scara_rviz.launch.py use_sim:=true plot:=true
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution, OrSubstitution
from launch_ros.actions import Node


def get_robot_description(xacro_path: str) -> str:
    result = subprocess.run(
        ["xacro", xacro_path], capture_output=True, text=True, check=True
    )
    return result.stdout


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("robnux_arm_sim")
    xacro_path = os.path.join(pkg, "urdf", "scara_robot.urdf.xacro")
    rviz_cfg = os.path.join(pkg, "config", "arm_sim.rviz")

    robot_description = get_robot_description(xacro_path)

    use_gui = LaunchConfiguration("use_gui", default="false")
    use_sim = LaunchConfiguration("use_sim", default="false")
    plot = LaunchConfiguration("plot", default="false")

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
            "plot",
            default_value="false",
            description="Launch rqt_plot for Cartesian position and orientation time-series plots",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        # Only start joint_state_publisher when neither use_gui nor use_sim is true
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{"use_gui": False}],
            condition=UnlessCondition(OrSubstitution(use_gui, use_sim)),
        ),

        # Only start joint_state_publisher_gui when use_gui=true and use_sim=false
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

        # rqt_plot for time-series: position (x/y/z) and orientation (roll/pitch/yaw)
        ExecuteProcess(
            cmd=[
                "/opt/ros/jazzy/lib/rqt_plot/rqt_plot",
                "/cart_pose/pose/position/x",
                "/cart_pose/pose/position/y",
                "/cart_pose/pose/position/z",
                "/arm_rpy/vector/x",
                "/arm_rpy/vector/y",
                "/arm_rpy/vector/z",
            ],
            output="screen",
            condition=IfCondition(plot),
        ),
    ])
