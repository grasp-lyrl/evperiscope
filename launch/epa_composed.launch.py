"""
Launch both epa_controller and epa_interface as composable nodes in a single process.

This enables zero-copy intra-process communication between the two nodes.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace, LoadComposableNodes
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
    
    dv_config_dir = os.path.join(get_package_share_directory('dv_ros2_capture'), 'config')
    dv_settings_file = os.path.join(dv_config_dir, 'settings.yaml')
    dv_dynamic_file = os.path.join(dv_config_dir, 'dynamic.yaml')

    # KR interface arguments
    kr_args = [
        DeclareLaunchArgument('robot', default_value='proteus'), # set robot namespace
        DeclareLaunchArgument('zed_enable', default_value='true'),
        DeclareLaunchArgument('ukf_enable', default_value='false'),
        DeclareLaunchArgument('visualization_enable', default_value='false'),
    ]
    
    # URDF/xacro file to be loaded by the Robot State Publisher node
    default_xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf',
        'zed_descr.urdf.xacro'
    )

    # ZED camera arguments
    zed_args = [        
        DeclareLaunchArgument('camera_name', default_value='zed'),
        DeclareLaunchArgument('camera_model', default_value='zedm'),
        DeclareLaunchArgument('publish_urdf', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('publish_map_tf', default_value='true'),
        DeclareLaunchArgument('publish_imu_tf', default_value='true'),
        DeclareLaunchArgument('xacro_path', default_value=TextSubstitution(text=default_xacro_path)),
        DeclareLaunchArgument('custom_baseline', default_value='0.0'),
        DeclareLaunchArgument('enable_gnss', default_value='false'),
        DeclareLaunchArgument('publish_svo_clock', default_value='false'),
    ]

    # Initialize launch description with all arguments
    ld = LaunchDescription([calib_arg] + kr_args + zed_args)

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
            # dvs driver
            ComposableNode(
                package='dv_ros2_capture',
                plugin='dv_capture_node::CaptureNode',
                name='capture_node',
                parameters=[dv_settings_file, dv_dynamic_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                condition=IfCondition(LaunchConfiguration('visualization_enable')),
                package='dv_ros2_visualization',
                plugin='dv_visualization_node::VisualizationNode',
                name='visualization_node',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # zed driver
            ComposableNode(
                condition=IfCondition(LaunchConfiguration('zed_enable')),
                package="zed_components",
                plugin="stereolabs::ZedCamera",
                name="zed_node",
                namespace=LaunchConfiguration('robot'),
                parameters=[
                    [FindPackageShare('zed_wrapper').find('zed_wrapper'), '/config/', LaunchConfiguration('camera_model'), '.yaml'],
                    [FindPackageShare('zed_wrapper').find('zed_wrapper'), '/config/common_stereo.yaml'],
                    # Finally apply launch-specific overrides
                    {
                        'general.camera_name': LaunchConfiguration('camera_name'),
                        'general.camera_model': LaunchConfiguration('camera_model'),
                        'pos_tracking.publish_tf': LaunchConfiguration('publish_tf'),
                        'pos_tracking.publish_map_tf': LaunchConfiguration('publish_map_tf'), 
                        'sensors.publish_imu_tf': LaunchConfiguration('publish_imu_tf', default='true'),
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # rosbag recorder
            ComposableNode(
                package='rosbag2_composable_recorder',
                plugin='rosbag2_composable_recorder::ComposableRecorder',
                name='recorder',
                parameters=[{
                    'topics': [
                        "/events",
                        "/imu",
                        "/proteus/zed_node/rgb/color/rect/image",
                        "/proteus/zed_node/rgb/color/rect/camera_info",
                        "/proteus/zed_node/depth/depth_registered",
                        "/proteus/zed_node/depth/depth_registered/camera_info",
                        "/proteus/zed_node/pose",
                        "/proteus/zed_node/odom",
                        "/kestrel1/zed_node/rgb/color/rect/image/compressed",
                        "/kestrel1/zed_node/rgb/color/rect/camera_info",
                        "/kestrel1/zed_node/depth/depth_registered/compressedDepth",
                        "/kestrel1/zed_node/depth/depth_registered/camera_info",
                        "/kestrel1/zed_node/control_odom",
                        "/kestrel1/zed_node/pose",
                        "/kestrel1/zed_node/odom",
                        "/kestrel1/mavros/global_position/global",                        
                        "/proteus/mavros/global_position/global",
                        "/proteus/mavros/global_position/raw/fix",
                        "/proteus/mavros/global_position/compass_hdg",
                        "/ouster/points",
                        "/rko_lio/odom",
                        "/rko_lio/local_map",
                        "/ublox_gps_node/fix",
                        # "/proteus/zed_node/mapping/fused_cloud",
                        # "/kestrel1/zed_node/mapping/fused_cloud",

                    ],
                    'storage_id': 'mcap',
                    'record_all': False,
                    'disable_discovery': False,
                    'serialization_format': 'cdr',
                    'start_recording_immediately': False,
                    'bag_prefix': '/bags/epa_',
                }],
                remappings=[],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    ld.add_action(container)

    # robot state publisher to publish URDF and static transforms for zed
    ld.add_action(
        Node(
            condition=IfCondition(LaunchConfiguration('zed_enable')),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=[LaunchConfiguration('camera_name'), '_state_publisher'],
            namespace=LaunchConfiguration('robot'),
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('publish_svo_clock'),
                'robot_description': Command([
                    'xacro', ' ', LaunchConfiguration('xacro_path'),
                    ' camera_name:=', LaunchConfiguration('camera_name'),
                    ' camera_model:=', LaunchConfiguration('camera_model'),
                    ' custom_baseline:=', LaunchConfiguration('custom_baseline')
                ])
            }]
        )
    )

    return ld
