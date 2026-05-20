import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


def generate_launch_description():
  nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
  prefix_address = get_package_share_directory('diadem_navigation')

  use_sim_time = LaunchConfiguration('use_sim_time', default='True')
  exploration   = LaunchConfiguration('exploration',  default='False')

  param_dir = LaunchConfiguration(
      'params_file',
      default=os.path.join(prefix_address, 'config', 'nav2_params.yaml'))

  navigation_launch_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([nav2_launch_dir, '/navigation_launch.py']),
      launch_arguments={
          'use_sim_time': use_sim_time,
          'params_file':  param_dir,
          'autostart':    'True',
      }.items(),
  )

  return LaunchDescription([
    DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation (Gazebo) clock'),
    DeclareLaunchArgument(
        'exploration', default_value='False',
        description='Whether to run in SLAM/exploration mode'),
    DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(prefix_address, 'config', 'nav2_params.yaml'),
        description='Path to Nav2 params yaml'),
    navigation_launch_cmd,
  ])