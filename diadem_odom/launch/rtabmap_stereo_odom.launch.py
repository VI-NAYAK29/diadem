import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Define the directories and paths
    realsense_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')

    # Define parameters and remappings
    stereo_odom_parameters = [{
        'frame_id': 'base_link',
        'publish_tf': False,
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

    # RealSense node
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_launch_dir, 'rs_launch.py')),
        launch_arguments={
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '1',
            'enable_infra1': 'true',
            'enable_infra2': 'true',
            'enable_sync': 'true',
#            'rgb_camera.color_profile': '640,360,6',
            'depth_module.emitter_enabled': 'true',
            'pointcloud.enable': 'true',
        }.items()
    )

    return LaunchDescription([
        realsense_node,
        # Nodes to launch
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
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=['0', '0', '0', '0.0', '0', '0', 'camera_link', 'camera_odom_link']
        # ),
        Node(
            package='diadem_odom', executable='param_setter.py', output='screen'  # Adjust the package name and executable accordingly
        ),

    ])
