import os
import launch
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import launch_ros


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='diadem_gazebo').find('diadem_gazebo')

    worlds_dir = os.path.join(pkg_share, 'worlds')
    gz_bridge_core_config = os.path.join(pkg_share, 'config', 'gz_bridge_core.yaml')
    gz_bridge_scan_config = os.path.join(pkg_share, 'config', 'gz_bridge_scan.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    from launch.actions import OpaqueFunction

    def resolve_world_path(context, *args, **kwargs):
        world_name = LaunchConfiguration('world').perform(context)
        is_headless = LaunchConfiguration('headless').perform(context)
        
        if os.path.isabs(world_name):
            world_full_path = world_name
        else:
            world_full_path = os.path.join(worlds_dir, world_name)
        
        cmd = ['gz', 'sim', '-r']
        if is_headless.lower() in ['true', '1']:
            cmd.append('-s')
        cmd.extend([world_full_path, '--verbose', '1'])
        
        return [ExecuteProcess(
            cmd=cmd,
            output='screen'
        )]

    gz_sim = OpaqueFunction(function=resolve_world_path)

    # Core Parameter Bridge (odom, imu, clock, cmd_vel)
    gz_bridge_core = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_core',
        arguments=['--ros-args', '-p', ['config_file:=', gz_bridge_core_config]],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Scan Parameter Bridge (lidar)
    gz_bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_scan',
        arguments=['--ros-args', '-p', ['config_file:=', gz_bridge_scan_config]],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Flag to enable use_sim_time'
        ),
        DeclareLaunchArgument(
            name='headless',
            default_value='False',
            description='Run Gazebo headless (without GUI client)'
        ),
        DeclareLaunchArgument(
            name='world',
            default_value='nav2_test_world.sdf',
            description='World file name (relative to worlds/) or absolute path. '
                        'Options: nav2_test_world.sdf | open_field.sdf | room2.sdf'
        ),
        # Make all local .sdf/.world files discoverable by gz sim (Harmonic/Jazzy)
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', worlds_dir),
        gz_sim,
        gz_bridge_core,
        gz_bridge_scan,
    ])

