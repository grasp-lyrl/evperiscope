import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyACM0:921600',
    )
    
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        namespace='/proteus/mavros/',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'system_id': 31,
            'system_target_id': 131,
        }]
    )

    return LaunchDescription([
        fcu_url_arg,
        mavros_node
    ])
