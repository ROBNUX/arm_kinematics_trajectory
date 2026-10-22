#!/usr/bin/env python3
"""
ROS2 launch file for Quattro 4-axis RViz visualization

Usage:
    ros2 launch quattro_bot quattro_4_rviz.launch.py                 # model only
    ros2 launch quattro_bot quattro_4_rviz.launch.py use_sim:=true   # + motion demo
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate ROS2 launch description for Quattro 4-axis with RViz"""

    pkg_dir = get_package_share_directory('quattro_bot')
    urdf_path = os.path.join(pkg_dir, 'model', 'quattro_4_macro.xacro')
    rviz_config = os.path.join(pkg_dir, 'launch', 'config4.rviz')

    # robot_state_publisher needs plain URDF, so expand the xacro first
    robot_description = subprocess.run(
        ['xacro', urdf_path], capture_output=True, text=True, check=True).stdout

    use_sim = LaunchConfiguration('use_sim')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim', default_value='false',
            description='Run simulate_quattro.py --four, which publishes /joint_states'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # Holds the model at its zero pose until a simulation publishes
        # /joint_states; not started together with the simulation.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui': False}],
            condition=UnlessCondition(use_sim),
        ),

        # The motion engine stamps /cart_pose and /arm_path in base_link; the
        # kinematics base frame is link_0 here.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_alias',
            arguments=['--frame-id', 'link_0', '--child-frame-id', 'base_link'],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'robnux_arm_sim', 'simulate_quattro.py', '--four'],
            output='screen',
            condition=IfCondition(use_sim),
        ),
    ])
