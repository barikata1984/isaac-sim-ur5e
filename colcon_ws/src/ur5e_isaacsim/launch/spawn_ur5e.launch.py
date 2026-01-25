from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur5e_isaacsim',
            executable='spawn_ur5e_wrapper.sh',
            name='ur5e_spawner',
            output='screen',
        ),
    ])
