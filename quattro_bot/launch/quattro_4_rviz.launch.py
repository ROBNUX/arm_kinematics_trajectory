#!/usr/bin/env python3
"""
ROS2 launch file for Quattro 4-axis RViz visualization
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate ROS2 launch description for Quattro 4-axis with RViz"""
    
    # Get package share directory
    pkg_dir = get_package_share_directory('quattro_bot')
    model_dir = os.path.join(pkg_dir, 'model')
    config_dir = os.path.join(pkg_dir, 'launch')
    
    # Path to URDF
    urdf_path = os.path.join(model_dir, 'quattro_4_macro.xacro')
    rviz_config = os.path.join(config_dir, 'config4.rviz')
    
    # Launch description
    ld = LaunchDescription()
    
    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_path],
        name='robot_state_publisher'
    )
    
    # Joint State Publisher
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_gui': False, 'publish_default_positions': False}],
        name='joint_state_publisher'
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        name='rviz2'
    )
    
    ld.add_action(robot_state_pub)
    ld.add_action(joint_state_pub)
    ld.add_action(rviz)
    
    return ld
