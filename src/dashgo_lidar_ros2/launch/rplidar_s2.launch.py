import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("dashgo_lidar_ros2")
    default_params = os.path.join(pkg_share, "config", "rplidar_s2.yaml")

    params_file = LaunchConfiguration("params_file")
    publish_laser_tf = LaunchConfiguration("publish_laser_tf")
    base_frame = LaunchConfiguration("base_frame")
    laser_frame = LaunchConfiguration("laser_frame")
    laser_z = LaunchConfiguration("laser_z")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the lidar parameter file.",
            ),
            DeclareLaunchArgument(
                "publish_laser_tf",
                default_value="true",
                description="Publish the static transform from base_link to laser.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Parent frame for the lidar static transform.",
            ),
            DeclareLaunchArgument(
                "laser_frame",
                default_value="laser",
                description="Laser frame id.",
            ),
            DeclareLaunchArgument(
                "laser_z",
                default_value="0.18",
                description="Laser height above base frame in meters.",
            ),
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_node",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_tf",
                condition=IfCondition(publish_laser_tf),
                arguments=[
                    "--x", "0.0",
                    "--y", "0.0",
                    "--z", laser_z,
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", "0.0",
                    "--frame-id", base_frame,
                    "--child-frame-id", laser_frame,
                ],
            ),
        ]
    )
