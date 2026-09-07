from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    
    # Launch first file immediately
    launch1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("diadem_bringup"),
                'launch',
                'state.launch.py'
            ])
        ])
    )
    
    # Launch second file after 5 seconds
    launch2 = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("diadem_bringup"),
                        'launch',
                        'depth_odom.launch.py'
                    ])
                ])
            )
        ]
    )
    
    # Launch third file after 10 seconds
    launch3 = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("diadem_navigation"),
                        'launch',
                        'nav3.launch.py'
                    ])
                ])
            )
        ]
    )
    
    # Launch robot_pose.py script after 3 seconds
    robot_pose_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='diadem_bringup',
                executable='robot_pose.py',
                name='robot_pose_node',
                output='screen'
            )
        ]
    )
    
    # ROS TCP endpoint node
    ros_tcp_endpoint_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_server',
        output='screen',
        parameters=[{'ROS_IP': '192.168.1.100'}]
    )
    
    return LaunchDescription([
        launch1,
        launch2,
        launch3,
        robot_pose_node,
        #ros_tcp_endpoint_node
    ])