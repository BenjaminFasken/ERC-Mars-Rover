import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    os.environ['ROS_DOMAIN_ID'] = '69'
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
    
    # # 2. Launch ZED2i Camera
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
    #         'general.resolution': '3',  # Change to VGA (lower than HD720)
    #         'general.fps': '5',
    #         'depth.depth_mode': '0',  # PERFORMANCE mode
    #         'depth.min_depth': '0.45',  # Increase minimum depth
    #         'depth.max_depth': '2.5',  # Decrease maximum depth range
    #         'depth.confidence': '90',  # Lower confidence threshold
    #         'depth.texture_confidence': '90',  # Higher texture confidence threshold
    #         'pos_tracking.imu_fusion': 'true',  # Enable IMU fusion for better performance
    #         }.items()
        
    #     )
    
    # Instead of using an event handler, use a timer to delay the startup
    # This waits for the ZED camera to fully initialize before starting dependent nodes
    delayed_actions = TimerAction(
        period=15.0,  # Allow 15 seconds for ZED camera initialization
        actions=[
            LogInfo(msg='Starting dependent nodes after ZED camera initialization delay...'),
            
            # # 3. Publish Static Transforms
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0.1646940', '-0.0598990', '0.2339370', '0', '0.7854', '0', 'base_link', 'zed_left_camera_frame']
            ),

                   

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0.082332', '0.000101', '0.160771', '0', '-0.17453', '-3.1416', 'base_link', 'livox_frame']
            ),

            # 4. Synchronize ZED2i RGB-D Data
            Node(
                package='rtabmap_sync', executable='rgbd_sync', output='screen',
                parameters=[{
                    'approx_sync':False
                }],
                remappings=[
                    ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
                    ('depth/image', '/zed/zed_node/depth/depth_registered'),
                    ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
                    ('rgbd_image', '/rgbd_image')
                ]
            ),
            Node(
                package='rtabmap_slam', executable='rtabmap', output='screen',
                parameters=[{

                'frame_id': 'base_link',
                'subscribe_scan_cloud':True,
                'subscribe_rgbd':True,
                'approx_sync':True,
                'wait_for_transform':0.2,
            
                'RGBD/ProximityMaxGraphDepth': '0',
                'RGBD/ProximityPathMaxNeighbors': '1',
                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',
                'RGBD/CreateOccupancyGrid': 'True',
                'Grid/Sensor': '2',
                'Mem/NotLinkedNodesKept': 'false',
                'Mem/STMSize': '30',
                'Mem/LaserScanNormalK': '20',
                'Reg/Strategy': '1',
                'Icp/VoxelSize': '0.1',
                'Icp/RangeMin': '0.3',
                'Icp/RangeMax': '4',
                'Grid/RangeMax': '4',
                'Icp/PointToPlaneK': '20',
                'Icp/PointToPlaneRadius': '0',
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '10',
                'Icp/Epsilon': '0.001',
                'Icp/MaxTranslation': '2',
                'Icp/MaxCorrespondenceDistance': '1',
                'Icp/Strategy': '1',
                'Icp/OutlierRatio': '0.7',
                'Icp/CorrespondenceRatio': '0.1',
                #
                'Grid/PreVoxelFiltering':  "true",
                'Grid/FlatObstacleDetected': "false",
                'Grid/NormalK': '20',
                'Grid/MaxObstacleHeight': '0.5',
                'Grid/MinGroundHeight': '0',
                'Grid/MaxGroundAngle': '25.0',
         
                'Grid/NormalSegments': 'true',
                'Grid/ClusterRadius': '0.1',
                'Grid/MinClusterSize': '10',
                'Grid/CellSize': '0.05',
                'Grid/NoiseFilteringRadius': '0.1',
                'Grid/RayTracing': 'true',
                #'Rtabmap/TimeThr': "15000", 
                # 'GFTT/'

                #
                # 'Grid/MaxObstacleHeight': '0.4',
                # 'Grid/MinObstacleHeight': '-0.02',

                # #
                # 'Grid/MaxGroundAngle': '25.0',
                # 'Grid/NormalK': '20',
                # 'Grid/MaxObstacleHeight': '0.5',
                
                # 'Grid/ProbHit': '0.7',
                # 'Grid/ProbMiss': '0.4',
                # 'Grid/ClusterRadius': '0.1',
                # 'Grid/RangeMax': '5.0',
                #
                }],
                remappings=[
                    ('scan_cloud', 'assembled_cloud')
                ],
                arguments=[
                    '-d'
             ]), 
            
            Node(
                package='rtabmap_odom', executable='icp_odometry', output='screen',
                parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id':'odom',
                'wait_for_transform':0.2,
                'expected_update_rate':15.0,
                'deskewing':deskewing,
                # RTAB-Map's internal parameters are strings:
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '10',
                'Icp/VoxelSize': '0.1',
                'Icp/Epsilon': '0.001',
                'Icp/PointToPlaneK': '40',
                'Icp/PointToPlaneRadius': '0',
                'Icp/MaxTranslation': '2',
                'Icp/MaxCorrespondenceDistance': '2',
                'Icp/RangeMin': '0.325', # Ignore laser scan points on the robot itself
                'Icp/RangeMax': '6',
                'Icp/Strategy': '1',
                'Icp/OutlierRatio': '0.7',
                'Icp/CorrespondenceRatio': '0.2',
                'Odom/ScanKeyFrameThr': '0.4',
                'OdomF2M/ScanSubtractRadius': '0.1',
                'OdomF2M/ScanMaxSize': '15000',
                'OdomF2M/BundleAdjustment': 'false',
                'Odom/GuessMotion': 'true',
                'Odom/ResetCountdown': '1',     # Reset after x failures
                }],
                remappings=[
                ('scan_cloud', '/livox/lidar'),
                ]),

            Node(
                package='rtabmap_util', executable='point_cloud_assembler', output='screen',
                parameters=[{
                'max_clouds':10,
                'fixed_frame_id':'',
                }],
                remappings=[
                ('cloud', 'odom_filtered_input_scan')
                ]),
            Node(
                package='rtabmap_viz', executable='rtabmap_viz', output='screen',
                parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id':'odom',
                'subscribe_rgbd':True,
                'subscribe_odom_info':True,
                'subscribe_scan_cloud':True,
                'approx_sync':False
                }],
                remappings=[
                ('scan_cloud', 'odom_filtered_input_scan')
                ]),
            
            
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
        #zed_wrapper_node,  # Launch the ZED camera
        delayed_actions    # Launch dependent nodes after a delay
    ])