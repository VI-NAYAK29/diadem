from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
       Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[{
                'gcs_url': 'udp-b://:14550@',
                'fcu_url': '/dev/ttyACM0:921600'
            }]
        ),
       Node(
           package='diadem_firmware',
           executable='pixhawk_to_cmd.py',
           output='screen'
       ),

    ])
