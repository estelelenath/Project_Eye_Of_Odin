# observation_program/launch/observation_program.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='observer',
            executable='observer',
            name='observer_node',
            output='screen',
        ),
    ])
