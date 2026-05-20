import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

gps_wpf_dir = get_package_share_directory("diadem_description")
mapviz_config_file = os.path.join(gps_wpf_dir, "config", "my.mvc")


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            parameters=[
                {"config": mapviz_config_file},
                {"use_sim_time": False}
            ]
        ),
        launch_ros.actions.Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            parameters=[
                {"use_sim_time": False},
                # {"local_xy_frame": "map"},
                # {"local_xy_origin": "custom"},  # Important: use "custom" instead of "auto"
                # {"local_xy_origin_latitude": 18.62},  # Use coordinates from your navsat logs
                # {"local_xy_origin_longitude": 73.91},
                # {"local_xy_origin_altitude": 558.16},
                # {"local_xy_origin_zone": "43Q"},
            ],
            remappings=[
                ("fix", "/gps/filtered"),
            ],
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "origin"],
            parameters=[
                {"use_sim_time": False}
            ]
        ),
        # launch_ros.actions.Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name="wgs84_to_utm_transform",
        #     arguments=["0", "0", "0", "0", "0", "0", "wgs84", "utm"],
        #     parameters=[
        #         {"use_sim_time": False}
        #     ]
        # )
    ])