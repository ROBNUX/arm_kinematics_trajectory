#!/usr/bin/env python3
"""
ROS2 launch file for Quattro test node
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate ROS2 launch description for Quattro test"""
    
    # Test node
    test_quattro = Node(
        package='quattro_bot',
        executable='test_quattro_rviz',
        name='quattro',
        respawn=True
    )
    
    ld = LaunchDescription()
    ld.add_action(test_quattro)
    
    return ld
