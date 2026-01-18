#!/usr/bin/env python3
"""
ROS2 Launch file for target position publisher.

Usage:
    ros2 launch target_publisher.launch.py
    ros2 launch target_publisher.launch.py target_topic:=/custom_target
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    # Get the package directory
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    scripts_dir = os.path.join(pkg_dir, 'scripts')
    
    # Declare launch arguments
    target_topic_arg = DeclareLaunchArgument(
        'target_topic',
        default_value='/target_position',
        description='Topic name for target position'
    )
    
    # Get launch configurations
    target_topic = LaunchConfiguration('target_topic')
    
    # Target publisher node
    # Uses standard ROS2 Python (requires ROS2 environment to be sourced)
    target_publisher = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(scripts_dir, 'target_publisher.py'),
        ],
        name='target_publisher',
        output='screen',
    )

    return LaunchDescription([
        # Arguments
        target_topic_arg,
        
        # Nodes
        target_publisher,
    ])
