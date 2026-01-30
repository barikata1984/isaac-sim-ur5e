"""Bring up launch file for UR robot simulation.

Launches Isaac Sim core with robot spawning enabled.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    """Generate launch description for UR robot simulation."""

    # Get the path to the isaac_sim_gui.sh script
    core_prefix = get_package_prefix('core')
    isaac_sim_script = os.path.join(core_prefix, 'lib', 'core', 'isaac_sim_gui.sh')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'robot_type',
            default_value='ur5e',
            description='Type of UR robot: ur5e, ur3, ur10, ur10e'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Isaac Sim in headless mode (true/false)'
        ),
        DeclareLaunchArgument(
            'prim_path',
            default_value='/World/UR',
            description='USD prim path for the robot'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='ur_robot',
            description='Name identifier for the robot'
        ),

        # Launch Isaac Sim using the wrapper script
        ExecuteProcess(
            cmd=[
                isaac_sim_script,
                '--ros-args',
                '-p', ['robot_type:=', LaunchConfiguration('robot_type')],
                '-p', ['headless:=', LaunchConfiguration('headless')],
                '-p', ['prim_path:=', LaunchConfiguration('prim_path')],
                '-p', ['robot_name:=', LaunchConfiguration('robot_name')],
            ],
            output='screen',
            additional_env={
                'ISAAC_HEADLESS': LaunchConfiguration('headless')
            },
        ),
    ])
