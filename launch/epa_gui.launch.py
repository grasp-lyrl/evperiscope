from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='epa',
            executable='epa_gui.py',
            name='epa_gui',
            output='screen',
        ),
    ])
