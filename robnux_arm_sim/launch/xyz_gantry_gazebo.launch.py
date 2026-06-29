#!/usr/bin/env python3
"""
Launch XYZ Gantry Cartesian robot in Gazebo.

Usage:
    ros2 launch robnux_arm_sim xyz_gantry_gazebo.launch.py
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def get_robot_description(xacro_path: str) -> str:
    result = subprocess.run(
        ["xacro", xacro_path], capture_output=True, text=True, check=True
    )
    return result.stdout


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("robnux_arm_sim")
    xacro_path = os.path.join(pkg, "urdf", "xyz_gantry_robot.urdf.xacro")
    sdf_world = os.path.join(pkg, "sdf", "xyz_gantry_world.sdf")

    robot_description = get_robot_description(xacro_path)

    return LaunchDescription([
        ExecuteProcess(
            cmd=["gz", "sim", sdf_world],
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    name="spawn_gantry",
                    arguments=[
                        "-name", "xyz_gantry",
                        "-string", robot_description,
                        "-x", "0", "-y", "0", "-z", "0",
                    ],
                    output="screen",
                ),
            ],
        ),

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
