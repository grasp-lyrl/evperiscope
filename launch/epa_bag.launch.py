"""Launch epa_controller alone, for replaying a recorded bag.

The event source is expected to come from `ros2 bag play` rather than a live
camera, so no driver nodes are started here.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory('epa')
    epa_controller_config = os.path.join(pkg_share, 'config', 'epa_controller.yaml')

    # Resolved from the installed package so the repo works from a fresh clone.
    # Override with calib_file:=/path/to.xml on the command line.
    calib_arg = DeclareLaunchArgument(
        'calib_file',
        default_value=os.path.join(pkg_share, 'config', 'calib_epa_60deg.xml'),
        description='OpenCV FileStorage XML camera calibration')

    ld = LaunchDescription([calib_arg])
    
    container = ComposableNodeContainer(
        name='epa_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # epa
            ComposableNode(
                package='epa',
                plugin='epa::EpaController',
                name='epa_controller',
                parameters=[epa_controller_config,
                            {'calib_file': LaunchConfiguration('calib_file')}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    ld.add_action(container)

    return ld
