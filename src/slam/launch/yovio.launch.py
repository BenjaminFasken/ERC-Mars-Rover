import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    #!!# Set the ROS_DOMAIN_ID to 69 fordi
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
    static_transform_publisher_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.1646940', '-0.0598990', '-0.2339370', '0', '-0.7854', '0', 'zed_camera_link', 'base_link']
    )
    
    static_transform_publisher_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.082332', '0.000101', '0.160771', '0', '-0.17453', '-3.1416', 'base_link', 'livox_frame']
    )
    # 2. Launch ZED2i Camera
    # zed_wrapper_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('zed_wrapper'),
    #             'launch',
    #             'zed_camera.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'camera_model': 'zed2i',
    #         'general.resolution': '2',  # Change to VGA (lower than HD720)
    #         'general.fps': '30',
    #         'depth.depth_mode': '0',  # PERFORMANCE mode
    #         'depth.min_depth': '0.30',  # Increase minimum depth
    #         'depth.max_depth': '4.0',  # Decrease maximum depth range
    #         'depth.confidence': '70',  # Lower confidence threshold
    #         'depth.texture_confidence': '70',  # Higher texture confidence threshold
    #         'pos_tracking.imu_fusion': 'true',  # Enable IMU fusion for better performance
    #         }.items()
        
    #     )

    
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
                'sync_queue_size': 20,
                'topic_queue_size': 20,
                'wait_for_transform': 0.5,
                'Grid/FootprintLength': '0.45',  
                'Grid/FootprintWidth': '0.45',   
                'Grid/FootprintHeight': '0.40', 
                'RGBD/ProximityMaxGraphDepth': '0',
                'RGBD/ProximityPathMaxNeighbors': '1',
                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',
                'RGBD/CreateOccupancyGrid': 'True',
                'Grid/Sensor': '0',
                'Mem/NotLinkedNodesKept': 'false',
                'Mem/STMSize': '30',
                'Mem/LaserScanNormalK': '20',
                'Reg/Strategy': '1',
                'Icp/VoxelSize': '0.025',
                'Icp/RangeMin': '0.35',
                'Icp/RangeMax': '12',
                'Grid/RangeMax': '12',
                'Icp/PointToPlaneK': '20',
                'Icp/PointToPlaneRadius': '0',
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '10',
                'Icp/Epsilon': '0.001',
                'Icp/MaxTranslation': '2',
                'Icp/MaxCorrespondenceDistance': '1',
                'Icp/Strategy': '0',
                'Icp/OutlierRatio': '0.7',
                'Icp/CorrespondenceRatio': '0.1',
                'Grid/PreVoxelFiltering': 'true',
                'Grid/FlatObstacleDetected': 'false',
                'Grid/NormalK': '20',
                'Grid/MaxObstacleHeight': '0.5',
                'Grid/MinGroundHeight': '0',
                'Grid/MaxGroundAngle': '25.0',
                'Grid/NormalSegments': 'true',
                'Grid/ClusterRadius': '0.1',
                'Grid/MinClusterSize': '10',
                'Grid/CellSize': '0.025',
                'Grid/NoiseFilteringRadius': '0.1',
                'Grid/NoiseFilteringMinNeighbors': '10',
                'Grid/RayTracing': 'true',
            }],
            remappings=[
                ('rgbd_image', '/rgbd_image'),
                ('odom', '/odom'),
                ('scan_cloud', '/livox/lidar'),  # LiDAR integration
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
        #         ('odom', '/odom'),
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
        #zed_wrapper_node,  # Launch the ZED camera
        rgbd_sync_node,
        delayed_actions    # Launch dependent nodes after a delay
    ])
