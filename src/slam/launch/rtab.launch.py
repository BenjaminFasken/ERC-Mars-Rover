import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    deskewing = LaunchConfiguration('deskewing')
    
    DeclareLaunchArgument(
        'deskewing', default_value='false',
        description='Enable lidar deskewing'),
    
    
    # 1. Launch Livox Driver
    livox_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam'),
                'launch',
                'msg_MID360_launch.py'
            ])
        ])
    )
    
    # 2. Launch sync node for ZED camera
    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        parameters=[{'approx_sync': True,
                    'approx_sync_max_interval': 0.05,
                    }],  
        remappings=[
            ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
            ('depth/image', '/zed/zed_node/depth/depth_registered'),
            ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
            ('rgbd_image', '/rgbd_image')
        ]
    )
    
    # 3. Launch static transform publishers
    static_transform_publisher_1 = Node(
        package='tf2_ros',
        namespace='tf2',
        name='camera_to_base_link_transform',
        executable='static_transform_publisher',
        arguments=['-0.147499', '-0.0598990', '-0.238857', '0', '-0.34906585', '0', 'zed_camera_link', 'base_link'],#x value differs from the report, but was experimentet to match better with this value.
        output='screen'  # Ensure logs are visible
    )
    
    static_transform_publisher_2 = Node(
        package='tf2_ros',
        name='base_link_to_livox_transform',
        executable='static_transform_publisher',
        arguments=['0.082332', '0.000101', '0.160771', '0', '-0.17453', '-3.1416', 'base_link', 'livox_frame']
    )
    

    delayed_actions = TimerAction(
    period=25.0,
    actions=[
        LogInfo(msg='Starting dependent nodes after ZED camera initialization delay...'),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,            # publishes odom → base_link
                'publish_tf_map_to_odom': True,# publishes map → odom
                'subscribe_scan_cloud': True,  # Enable LiDAR
                'subscribe_rgbd': True,
                'approx_sync': True,
                'sync_queue_size': 10, # Reduced from 20
                'topic_queue_size': 10, # Reduced from 20
                'wait_for_transform': 2.0,
                'Grid/FootprintLength': '0.5',
                'Grid/FootprintWidth': '0.5',
                'Grid/FootprintHeight': '0.45',
                'Grid/RangeMin': '0.35',
                'RGBD/ProximityMaxGraphDepth': '0',
                'RGBD/ProximityPathMaxNeighbors': '1',
                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',
                'RGBD/CreateOccupancyGrid': 'True',
                'Grid/Sensor': '2',   # 0: LiDAR, 1: RGBD, 2: Stereo
                'Mem/NotLinkedNodesKept': 'false',
                'Mem/STMSize': '15', # Reduced from 30
                'Mem/LaserScanNormalK': '20',
                'Reg/Strategy': '2',
                'Grid/RangeMax': '40',
                'Grid/PreVoxelFiltering': 'true',
                'Grid/FlatObstacleDetected': 'false',
                'Grid/NormalK': '10',
                # 'Grid/MaxObstacleHeight': '0.5',
                # 'Grid/MinGroundHeight': '0',
                'Grid/MaxGroundAngle': '25.0',
                'Grid/NormalSegments': 'true',
                'Grid/ClusterRadius': '0.1',
                'Grid/MinClusterSize': '10',
                'Grid/CellSize': '0.04', # Increased from 0.037
                'Grid/NoiseFilteringRadius': '0.15',
                'Grid/NoiseFilteringMinNeighbors': '32',
                'Grid/RayTracing': 'false', # Disabled from true
            }],
            remappings=[
                ('rgbd_image', '/rgbd_image'),
                ('scan_cloud', '/livox/lidar'),  # LiDAR integration
                # ('map', 'map2'),
                # ('/map', '/map2')
            ],
            arguments=['-d']
        ),
        # Node(
        #     package='rtabmap_viz',
        #     executable='rtabmap_viz',
        #     output='screen',
        #     parameters=[{
        #         'frame_id': 'base_link',
        #         'odom_frame_id': 'odom',
        #         'subscribe_rgbd': True,
        #         'subscribe_odom_info': True,
        #         'subscribe_scan_cloud': True,  # Optional: Enable for visualization
        #         'approx_sync': False
        #     }],
        #     remappings=[
        #         ('rgbd_image', '/rgbd_image'),
        #         ('scan_cloud', '/livox/lidar'),
        #     ]
        # ),
    ]
)
    # Create and return launch description
    return LaunchDescription([
        LogInfo(msg='Starting combined Livox LiDAR and ZED2i Camera with RTAB-Map...'),
        DeclareLaunchArgument(
            'deskewing', default_value='false',
            description='Enable lidar deskewing'
        ),
        livox_driver_launch,
        static_transform_publisher_1,
        static_transform_publisher_2,
        rgbd_sync_node,
        delayed_actions    # Launch dependent nodes after a delay
    ])