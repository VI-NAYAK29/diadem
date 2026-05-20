import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav_pkg_share = get_package_share_directory('diadem_navigation')
    nav2_bringup_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file  = LaunchConfiguration('params_file', default=os.path.join(nav_pkg_share, 'config', 'nav2_params.yaml'))
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(nav_pkg_share, 'maps', 'nav2_test_map.yaml'))

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml_file}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(nav_pkg_share, 'maps', 'nav2_test_map.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(nav_pkg_share, 'config', 'nav2_params.yaml')),

        map_server_node,
        amcl_node,
        navigation_launch_cmd,
        lifecycle_manager_node
    ])
