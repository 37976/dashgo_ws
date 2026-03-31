from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("dashgo_driver_ros2"),
        "config",
        "my_dashgo_params.yaml",
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the ROS 2 parameters file.",
            ),
            Node(
                package="dashgo_driver_ros2",
                executable="dashgo_driver_node",
                name="dashgo_driver",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
