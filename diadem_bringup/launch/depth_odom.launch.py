import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    navigation_dir = os.path.join(get_package_share_directory('diadem_navigation'), 'launch')
    odom_launch_dir = os.path.join(get_package_share_directory('diadem_odom'), 'launch')
    map_directory = os.path.join(get_package_share_directory('diadem_navigation'), 'maps', 'dock.yaml')

    

    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'navigation.launch.py'))
                          )
    
    rtabmap_odometry_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odom_launch_dir, 'rtabmap_stereo_odom.launch.py')))

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odom_launch_dir, 'ekf.launch.py')))


    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(name='exploration', default_value='true',
                                             description='Flag to enable exploration'),
        DeclareLaunchArgument(name='map_file', default_value=map_directory,
                                              description='Map to be used'),
        DeclareLaunchArgument(name='realsense', default_value='False',
                                              description='Realsense to be used'),
        DeclareLaunchArgument(name='joy', default_value='True',
                                              description='To enable joystick control'),
        # state_publisher_launch_cmd,
        #navigation_launch_cmd,
        rtabmap_odometry_launch_cmd,
        ekf,
        # navigation_launch_cmd,
        #slam_toolbox_launch_cmd,
        # microros_node,
        # pf_launch,


    ])
