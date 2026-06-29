#!/usr/bin/env python3
"""
Launch SCARA robot in Gazebo (Ignition/gz-sim).

Usage:
    ros2 launch robnux_arm_sim scara_gazebo.launch.py
    ros2 launch robnux_arm_sim scara_gazebo.launch.py headless:=true

Prerequisites (ROS2 Humble + Gazebo Fortress or later):
    sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_robot_description(xacro_path: str) -> str:
    result = subprocess.run(
        ["xacro", xacro_path], capture_output=True, text=True, check=True
    )
    return result.stdout


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("robnux_arm_sim")
    xacro_path = os.path.join(pkg, "urdf", "scara_robot.urdf.xacro")
    sdf_world = os.path.join(pkg, "sdf", "scara_world.sdf")

    robot_description = get_robot_description(xacro_path)

    headless = LaunchConfiguration("headless", default="false")

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo without GUI (server-only mode)",
        ),

        # Launch Gazebo with the scara world
        ExecuteProcess(
            cmd=[
                "gz", "sim", sdf_world,
                "--headless-rendering",
            ],
            condition=IfCondition(headless),
            output="screen",
        ),

        ExecuteProcess(
            cmd=["gz", "sim", sdf_world],
            output="screen",
        ),

        # robot_state_publisher provides the TF tree
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        # Spawn the URDF into Gazebo after a short delay
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    name="spawn_scara",
                    arguments=[
                        "-name", "scara_robot",
                        "-string", robot_description,
                        "-x", "0", "-y", "0", "-z", "0.4",
                    ],
                    output="screen",
                ),
            ],
        ),

        # Bridge /joint_states from Gazebo to ROS2
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_ros2_bridge",
            arguments=[
                "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            ],
            output="screen",
        ),
    ])
