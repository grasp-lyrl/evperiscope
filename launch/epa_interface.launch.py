from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='epa_interface_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='epa',
                plugin='epa::EpaInterface',
                name='epa_interface',
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
