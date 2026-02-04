#!/usr/bin/env python3
"""
ROS2 launch file for Quattro RViz test
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate ROS2 launch description for Quattro RViz test"""
    
    # Test node
    test_quattro_rviz = Node(
        package='quattro_bot',
        executable='test_quattro_rviz',
        name='test_quattro_rviz',
        respawn=False
    )
    
    ld = LaunchDescription()
    ld.add_action(test_quattro_rviz)
    
    return ld
