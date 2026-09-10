import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # KR interface arguments
    kr_args = [
        DeclareLaunchArgument('robot', default_value='kestrel1'), # set robot namespace
        DeclareLaunchArgument('zed_enable', default_value='true'),
        DeclareLaunchArgument('ukf_enable', default_value='false'),
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
    ld = LaunchDescription(kr_args + zed_args)

    # Create a main component container for all composable nodes
    main_container = ComposableNodeContainer(
        name="control_container",
        namespace="",  # Empty namespace for container, individual nodes will have their own namespaces
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
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
            ComposableNode(
                condition=IfCondition(LaunchConfiguration('ukf_enable')),
                package="quadrotor_ukf",
                plugin="QuadrotorUkfNode",
                name="quadrotor_ukf",
                namespace=LaunchConfiguration('robot'),
                parameters=[{
                    'frame_id': 'odom',
                    'imu_frame_id': 'zed_imu_link',
                    'body_frame_id': 'zed_camera_link',
                }],
                remappings=[
                    ('imu', 'zed_node/imu/data'),
                    ('odom', 'zed_node/odom'),
                ],
            ),
            ComposableNode(
                package="epa",
                plugin="epa::EpaInterface",
                name="epa_interface",
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
    
    ld.add_action(main_container)
    
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

    # Pose to Odometry Republisher node -- note this just rebros pose, does not have velocity info
    input_topic_val = ['/', LaunchConfiguration('robot'), '/zed_node/pose']
    output_topic_val = ['/', LaunchConfiguration('robot'), '/zed_node/control_odom']

    pose_to_odom_republisher = Node(
        package='epa',
        executable='pose_to_odom_republisher.py',
        parameters=[{
            'input_topic': input_topic_val,
            'output_topic': output_topic_val,
            'child_frame_id': 'base_link'
        }]
    )

    ld.add_action(pose_to_odom_republisher)

    # mavros
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        namespace='/kestrel1/mavros/',
        parameters=[{
            'fcu_url': '/dev/ttyTHS1:921600'
        }]
    )
    ld.add_action(mavros_node)

    return ld
