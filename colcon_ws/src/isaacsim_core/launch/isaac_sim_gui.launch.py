from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaacsim_core',
            executable='isaac_sim_gui.sh',
            name='isaac_sim_gui',
            output='screen',
        ),
    ])
