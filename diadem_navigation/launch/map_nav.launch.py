import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
import launch
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
import launch_ros
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    # nav2_yaml = os.path.join(get_package_share_directory('acrux_navigation'), 'config', 'nav2_params_simulation.yaml')
    nav2_yaml =  '/home/rigbetellabs/gps_ws/src/diadem_navigation/config/map_nav_params.yaml' #os.path.join(get_package_share_directory('diadem_navigation'), 'config', 'map_nav_params.yaml')
    # nav2_yaml = '/home/rigbetel_labs/ros2_ws/src/Navigation-experiments-/nav2_params_simulation.yaml'
    map_file = '/home/rigbetellabs/gps_ws/src/diadem_navigation/maps/out_test3.yaml'
    # map_file = '/home/rigbetel_labs/vitesco_voyage_map2.yaml'
    # params_file = '/home/rigbetel_labs/ros2_ws/src/acrux_private/acrux_navigation/config/nav2_params_simulation.yaml'
  
    lifecycle_nodes = ['map_server'
                       ]
    # start_costmap_filter_info_server_cmd = launch_ros.actions.Node(
    #     package='nav2_map_server',
    #     executable='costmap_filter_info_server',
    #     name='costmap_filter_info_server',
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=[params_file])
    
    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_yaml,
            'map_file': map_file,  # Pass map_file argument
           
        }.items()
    )

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False}, 
                        {'yaml_filename':map_file}]),

        # Node(
        #     package='nav2_amcl',
        #     executable='amcl',
        #     name='amcl',
        #     output='screen',
        #     parameters=[nav2_yaml]),
            
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]),
        
        
        navigation_launch_cmd,
        # start_costmap_filter_info_server_cmd,
        
        
    ])