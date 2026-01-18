#!/usr/bin/env python3
"""
ROS2 Launch file for UR5e manipulator control in Isaac Sim.

Usage:
    ros2 launch ur5e_control.launch.py
    ros2 launch ur5e_control.launch.py manipulator:=ur10e headless:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

import os


def generate_launch_description():
    # Get the package directory
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    scripts_dir = os.path.join(pkg_dir, 'scripts')
    
    # Declare launch arguments
    manipulator_arg = DeclareLaunchArgument(
        'manipulator',
        default_value='ur5e',
        description='Manipulator to control (ur5e, ur10e, etc.)'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run simulation in headless mode'
    )
    
    use_target_publisher_arg = DeclareLaunchArgument(
        'use_target_publisher',
        default_value='true',
        description='Launch the target position publisher node'
    )

    # Get launch configurations
    manipulator = LaunchConfiguration('manipulator')
    headless = LaunchConfiguration('headless')
    use_target_publisher = LaunchConfiguration('use_target_publisher')
    
    # Isaac Sim ROS2 control (runs via Isaac Sim's Python)
    # Note: Isaac Sim scripts need to be launched through its own Python environment
    isaac_sim_control = ExecuteProcess(
        cmd=[
            os.path.join(pkg_dir, 'scripts', 'run_ros2.sh'),
            os.path.join(scripts_dir, 'ros2_control.py'),
            '--manipulator', manipulator,
            '--headless', headless,
        ],
        name='isaac_sim_ros2_control',
        output='screen',
    )
    
    # Target publisher node (can run with standard ROS2 Python)
    # Uncomment if you want to run target_publisher as a separate node
    # target_publisher = Node(
    #     package='isaac_sim_ur5e',  # Requires proper package setup
    #     executable='target_publisher',
    #     name='target_publisher',
    #     output='screen',
    #     condition=IfCondition(use_target_publisher),
    # )

    return LaunchDescription([
        # Arguments
        manipulator_arg,
        headless_arg,
        use_target_publisher_arg,
        
        # Processes
        isaac_sim_control,
    ])
