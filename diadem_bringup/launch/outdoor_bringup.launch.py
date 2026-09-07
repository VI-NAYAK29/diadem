#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the ROS_IP launch argument
    # ros_ip_arg = DeclareLaunchArgument(
    #     'ros_ip',
    #     default_value='192.168.0.102',
    #     description='ROS IP address for TCP endpoint'
    # )
    
    # Define the esptool command
    esptool_cmd = ExecuteProcess(
       cmd=['esptool', '--port', '/dev/esp', 'read_mac'],
       name='esptool_read_mac',
       output='screen'
    )
    #   cmd=['python3', '/home/diadem/battery_bms_data.py'],
     #   name='bms_data',
      #  output='screen'
   # )
    # Define all the nodes that should start after esptool completes
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/esp', '-b', '921600'],
        output='screen'
    )
    
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        parameters=[
            #{'gcs_url': 'udp://:14550@192.168.0.103:14550'},
            {'gcs_url': 'udp-b://:14550@'},
            {'fcu_url': '/dev/px4:921600'}
        ],
        output='screen'
    )
    
    pixhawk_to_cmd_node = Node(
        package='diadem_firmware',
        executable='pixhawk_to_cmd.py',
        name='pixhawk_to_cmd',
        output='screen'
    )
    
    # ros_tcp_endpoint_node = Node(
    #     package='ros_tcp_endpoint',
    #     executable='default_server_endpoint',
    #     parameters=[
    #         {'ROS_IP': LaunchConfiguration('ros_ip')}
    #     ],
    #     output='screen',
    #     respawn=True,
    #     respawn_delay=2.0
    # )
    
    # pixhawk_imu_to_euler_node = Node(
    #     package='pixhawk_imu_to_euler',
    #     executable='pixhawk_imu_to_euler',
    #     output='screen'
    # )
    
    # Event handler to start all nodes after esptool completes
    start_nodes_after_esptool = RegisterEventHandler(
        OnProcessExit(
            target_action=esptool_cmd,
            on_exit=[
                micro_ros_agent_node,
                mavros_node,
                pixhawk_to_cmd_node,
                #ros_tcp_endpoint_node,
                #pixhawk_imu_to_euler_node,
                #bms_cmd
            ]
        )
    )
    
    return LaunchDescription([
                micro_ros_agent_node,
                mavros_node,
                #pixhawk_to_cmd_node,
    ])
