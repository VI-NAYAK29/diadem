import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess, DeclareLaunchArgument
import launch_ros
import os
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='diadem_description').find('diadem_description')
    gazebo_pkg_share = launch_ros.substitutions.FindPackageShare(
        package='diadem_gazebo').find('diadem_gazebo')

    default_model_path = os.path.join(pkg_share, 'urdf/diadem_sim.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/sensors.rviz')
    world_path = os.path.join(gazebo_pkg_share, 'worlds/room2.sdf')
    gz_bridge_core_config = os.path.join(gazebo_pkg_share, 'config/gz_bridge_core.yaml')
    gz_bridge_scan_config = os.path.join(gazebo_pkg_share, 'config/gz_bridge_scan.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('model')]),
                value_type=str)
        }]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    gz_sim = ExecuteProcess(
        condition=IfCondition(use_sim_time),
        cmd=['gz', 'sim', '-r', world_path, '--verbose', '1'],
        output='screen'
    )

    gz_bridge_core = launch_ros.actions.Node(
        condition=IfCondition(use_sim_time),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_core',
        arguments=['--ros-args', '-p', ['config_file:=', gz_bridge_core_config]],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    gz_bridge_scan = launch_ros.actions.Node(
        condition=IfCondition(use_sim_time),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_scan',
        arguments=['--ros-args', '-p', ['config_file:=', gz_bridge_scan_config]],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    spawn_entity = launch_ros.actions.Node(
        condition=IfCondition(use_sim_time),
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'diadem', '-topic', 'robot_description', '-z', '0.3'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', default_value='True',
            description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(
            name='model', default_value=default_model_path,
            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(
            name='rvizconfig', default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'),

        gz_sim,
        gz_bridge_core,
        gz_bridge_scan,
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node,
    ])
