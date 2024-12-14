from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab1_pkg',  # Your package name
            executable='talker',  # Entry point defined in setup.py
            name='talker_node',  # Name of the node (optional)
            output='screen',
        ),
        Node(
            package='lab1_pkg',
            executable='listener',
            name='relay_node',
            output='screen',
        ),
    ])
