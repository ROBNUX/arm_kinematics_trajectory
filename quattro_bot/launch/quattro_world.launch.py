#!/usr/bin/env python3
"""
ROS2 launch file for Quattro robot in Gazebo simulation
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate ROS2 launch description for Quattro simulation"""
    
    # Get package share directory
    pkg_dir = get_package_share_directory('quattro_bot')
    model_dir = os.path.join(pkg_dir, 'model')
    
    # Declare launch arguments
    paused = DeclareLaunchArgument(
        'paused', 
        default_value='false',
        description='Start the simulation in a paused state'
    )
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time instead of wall clock time'
    )
    
    gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI'
    )
    
    headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )
    
    debug = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Start Gazebo in debug mode'
    )
    
    # Include Gazebo empty world launch file
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'empty_world.launch.py'
            )
        ),
        launch_arguments={
            'debug': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'headless': LaunchConfiguration('headless'),
        }.items()
    )
    
    # Path to URDF
    urdf_path = os.path.join(model_dir, 'quattro_macro.xacro')
    
    # Launch description components
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(paused)
    ld.add_action(use_sim_time)
    ld.add_action(gui)
    ld.add_action(headless)
    ld.add_action(debug)
    
    # Include Gazebo launch
    ld.add_action(gazebo_empty_world)
    
    # Note: Robot state publisher and spawn_model should be added separately
    # This is a basic template - you may need to add additional nodes
    
    return ld
