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


    static_transform_publisher_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        #arguments=['-0.13278', '0.0', '-0.42577', '0', '0.0', '0', 'zed_camera_link', 'base_link']
        #arguments=['-0.13278', '0.0', '-0.22794', '0', '0.0', '0', 'zed_camera_link', 'base_link']
        arguments=['-0.14187', '0.05994', '-0.24134', '0', '0.0', '0', 'zed_camera_link', 'base_link']
    )
    
    static_transform_publisher_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        #arguments=['0.065', '0.0', '0.24783', '0', '0', '0', 'base_link', 'livox_frame']
        #arguments=['0.065', '0.0', '0.29783', '0', '0', '0', 'base_link', 'livox_frame']
        arguments=['0.065', '0.0', '0.1', '0', '0', '0', 'base_link', 'livox_frame']
        #arguments=['0.065', '0.0', '0.13622', '0', '0', '0', 'base_link', 'livox_frame']
    )
    

    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        parameters=[{'use_sim_time': True,
                    'approx_sync': True,
                    'approx_sync_max_interval': 0.4,
                    }],  
        remappings=[
            ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
            ('depth/image', '/zed/zed_node/depth/depth_registered'),
            ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
            ('rgbd_image', '/rgbd_image')
        ]
    )
    
    delay_rgbd_sync_node = TimerAction(
    period=0.0,
    actions=[
        LogInfo(msg='Starting rgbd_sync_node after initialization 10 s delay...'),
        rgbd_sync_node
    ]
)
    
    delayed_actions = TimerAction(
    period=10.0, #25.0,
    actions=[
        LogInfo(msg='Starting dependent nodes after ZED camera initialization delay...'),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,            # publishes odom → base_link
                'publish_tf_map_to_odom': False,# publishes map → odom
                'subscribe_scan_cloud': True,  # Enable LiDAR
                'subscribe_rgbd': True,
                'approx_sync': True,
                'sync_queue_size': 1, # Reduced from 20
                'topic_queue_size': 1, # Reduced from 20
                'wait_for_transform': 400.0, # Reduced from 0.5
                'Mem/DepthCompressionFormat':".png",
                'Grid/FootprintLength': '0.5',
                'Grid/FootprintWidth': '0.5',
                'Grid/FootprintHeight': '0.45',
                'Grid/RangeMin': '0.35',
                'RGBD/ProximityMaxGraphDepth': '0',
                'RGBD/ProximityPathMaxNeighbors': '1',
                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',
                'RGBD/CreateOccupancyGrid': 'True',
                'Grid/Sensor': '0',
                'Mem/NotLinkedNodesKept': 'false',
                'Mem/SaveDepth16Format' : "true",
                'Mem/STMSize': '50', # Reduced from 30
                'Mem/LaserScanNormalK': '20',
                'Reg/Strategy': '1',
                'Grid/RangeMax': '40',
                'Grid/PreVoxelFiltering': 'true',
                'Grid/FlatObstacleDetected': 'false',
                'Grid/NormalK': '10',
                'Grid/MaxObstacleHeight': '0.5',
                'Grid/MinGroundHeight': '0',
                'Grid/MaxGroundAngle': '25.0',
                'Grid/NormalSegments': 'true',
                'Grid/ClusterRadius': '0.1',
                'Grid/MinClusterSize': '1',
                'Grid/CellSize': '0.04', # Increased from 0.037
                'Grid/NoiseFilteringRadius': '0',
                'Grid/NoiseFilteringMinNeighbors': '1',
                'Grid/RayTracing': 'true', # Disabled from true
            }],
            remappings=[
                ('rgbd_image', '/rgbd_image'),
                #('odom', '/odom'),
                ('scan_cloud', '/livox/lidar'),  # LiDAR integration
            ],
            arguments=['-d']
        ),
    ]
)
    
    # Create and return launch description
    return LaunchDescription([
        LogInfo(msg='Starting combined Livox LiDAR and ZED2i Camera with RTAB-Map...'),
        DeclareLaunchArgument(
            'deskewing', default_value='false',
            description='Enable lidar deskewing'
        ),
        static_transform_publisher_1,
        static_transform_publisher_2,
        delay_rgbd_sync_node,
        delayed_actions    # Launch dependent nodes after a delay
    ])


