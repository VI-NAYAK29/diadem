import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
import launch_ros


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='diadem_description').find('diadem_description')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'diadem',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Flag to enable use_sim_time'
        ),
        launch.actions.DeclareLaunchArgument(
            name='x_pose', default_value='0.0',
            description='Spawn x position'),
        launch.actions.DeclareLaunchArgument(
            name='y_pose', default_value='0.0',
            description='Spawn y position'),
        launch.actions.DeclareLaunchArgument(
            name='z_pose', default_value='0.05',
            description='Spawn z position'),
        launch.actions.DeclareLaunchArgument(
            name='yaw_pose', default_value='0.0',
            description='Spawn yaw orientation'),

        spawn_robot,
    ])
