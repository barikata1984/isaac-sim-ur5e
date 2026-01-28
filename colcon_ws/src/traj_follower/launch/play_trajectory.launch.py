from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'json_path',
            default_value=os.path.join(
                os.getcwd(), 'src/trajectory/results/spline.json'
            ),
            description='Path to the trajectory JSON file'
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='false',
            description='Whether to loop the trajectory'
        ),
        Node(
            package='traj_follower',
            executable='follower_node',
            name='trajectory_follower',
            output='screen',
            emulate_tty=True, # Allows capturing stdin
            parameters=[{
                'json_path': LaunchConfiguration('json_path'),
                'loop': LaunchConfiguration('loop')
            }]
        )
    ])
