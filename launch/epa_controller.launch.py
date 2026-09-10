import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('epa')
    config = os.path.join(pkg_share, 'config', 'epa_controller.yaml')

    # The config file stays authoritative: its values become the launch argument
    # defaults, so a launch argument only takes effect when explicitly passed.
    with open(config) as f:
        params = yaml.safe_load(f)['epa_controller']['ros__parameters']

    args = [
        # Resolved from the installed package so the repo works from a fresh
        # clone. Override with calib_file:=/path/to.xml on the command line.
        DeclareLaunchArgument(
            'calib_file',
            default_value=params.get('calib_file')
            or os.path.join(pkg_share, 'config', 'calib_epa_60deg.xml'),
            description='OpenCV FileStorage XML camera calibration'),
        DeclareLaunchArgument(
            'input_mode',
            default_value=str(params.get('input_mode', 'topic')),
            description="'topic' for a live camera or bag playback, 'h5' to "
                        'replay an HDF5 event file'),
        DeclareLaunchArgument(
            'h5_filepath',
            default_value=str(params.get('h5_filepath', '')),
            description='HDF5 event file, used when input_mode is h5'),
        DeclareLaunchArgument(
            't_start',
            default_value=str(params.get('t_start', 15.0)),
            description='Start time within the recording, seconds (h5 mode)'),
    ]

    container = ComposableNodeContainer(
        name='epa_controller_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='epa',
                plugin='epa::EpaController',
                name='epa_controller',
                parameters=[config, {
                    'calib_file': LaunchConfiguration('calib_file'),
                    'input_mode': LaunchConfiguration('input_mode'),
                    'h5_filepath': LaunchConfiguration('h5_filepath'),
                    't_start': ParameterValue(LaunchConfiguration('t_start'),
                                              value_type=float),
                }],
            ),
        ],
        output='screen',
    )

    return LaunchDescription(args + [container])
