import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    realsense_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')

    # Parameters and remappings
    stereo_odom_parameters = [{
        'frame_id': 'base_link',
        'publish_tf': True,
        'subscribe_stereo': True,
        'subscribe_odom_info': True,
        'wait_imu_to_init': True,
        'pointcloud.enable': True,
    }]
    stereo_odom_remappings = [
        ('imu', '/camera_imu'),
        ('odom', '/odom'),
        ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
        ('left/camera_info', '/camera/camera/infra1/camera_info'),
        ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
        ('right/camera_info', '/camera/camera/infra2/camera_info')
    ]

    imu_filter_parameters = [{
        'use_mag': False,
        'world_frame': 'enu',
        'publish_tf': False
    }]
    imu_filter_remappings = [
        ('imu/data_raw', '/camera/camera/imu'),
        ('imu/data', '/camera_imu')
    ]

    # RTAB-Map SLAM parameters with 2D grid mapping enabled
    rtabmap_parameters = [{
        'frame_id': 'base_link',
        'subscribe_stereo': True,
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_odom_info': True,
        'subscribe_imu': True,
        'queue_size': 30,
        'approx_sync': True,
        'subscribe_scan': False,
        'publish_tf': True,
        'publish_map_graph': True,
        'use_action_for_goal': False,
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        
        # 2D Grid Map parameters
        'Grid/Enabled': 'true',
        'Grid/FromDepth': 'false',
        'Grid/RayTracing': 'true',
        'Grid/MaxObstacleHeight': '2.0',
        'Grid/MinGroundHeight': '-0.1',
        'Grid/MaxGroundHeight': '0.1',
        'Grid/CellSize': '0.05',
        'Grid/3D': 'false',
        'Grid/2D': 'true',
        'Grid/MapFrameProjection': 'true',
        
        # Mapping quality parameters
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'Mem/RehearsalSimilarity': '0.2',
        'Rtabmap/DetectionRate': '1',
    }]

    rtabmap_remappings = [
        ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
        ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
        ('left/camera_info', '/camera/camera/infra1/camera_info'),
        ('right/camera_info', '/camera/camera/infra2/camera_info'),
        ('odom', '/odom'),
        ('imu', '/camera_imu'),
        ('grid_map', '/map'),  # Publish grid map to the standard /map topic
    ]

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_launch_dir, 'rs_launch.py')),
        launch_arguments={
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '1',
            'enable_infra1': 'true',
            'enable_infra2': 'true',
            'enable_sync': 'true',
            'depth_module.emitter_enabled': 'true',
            'pointcloud.enable': 'true',
        }.items()
    )

    return LaunchDescription([
        realsense_node,

        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=stereo_odom_parameters,
            remappings=stereo_odom_remappings
        ),

        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=imu_filter_parameters,
            remappings=imu_filter_remappings
        ),

        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']
        ),

        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '1.571', '0', '0', 'base_link', 'camera_odom_link']
        ),

        Node(
            package='diadem_odom', executable='param_setter.py', output='screen'
        ),

        # RTAB-Map SLAM Node
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings
        ),
        
        # # Add rtabmap_viz node for visualization
        # Node(
        #     package='rtabmap_viz', 
        #     executable='rtabmap_viz', 
        #     output='screen',
        #     parameters=[{'frame_id': 'base_link'}],
        #     remappings=rtabmap_remappings
        # ),
    ])