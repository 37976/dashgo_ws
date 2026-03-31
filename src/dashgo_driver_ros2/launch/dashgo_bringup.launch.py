import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def static_tf_node(
    name,
    parent_frame,
    child_frame,
    x,
    y,
    z,
    roll,
    pitch,
    yaw,
    enabled,
):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        condition=IfCondition(enabled),
        arguments=[
            "--x", str(x),
            "--y", str(y),
            "--z", str(z),
            "--roll", str(roll),
            "--pitch", str(pitch),
            "--yaw", str(yaw),
            "--frame-id", parent_frame,
            "--child-frame-id", child_frame,
        ],
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("dashgo_driver_ros2")
    default_params = os.path.join(pkg_share, "config", "my_dashgo_params.yaml")

    params_file = LaunchConfiguration("params_file")
    publish_sonar_tf = LaunchConfiguration("publish_sonar_tf")
    base_frame = LaunchConfiguration("base_frame")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the ROS 2 parameters file.",
            ),
            DeclareLaunchArgument(
                "publish_sonar_tf",
                default_value="true",
                description="Publish static transforms for sonar frames.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_footprint",
                description="Parent frame for sonar transforms.",
            ),
            Node(
                package="dashgo_driver_ros2",
                executable="dashgo_driver_node",
                name="dashgo_driver",
                output="screen",
                parameters=[params_file],
            ),
            static_tf_node(
                "base_link_to_sonar0",
                base_frame,
                "sonar0",
                0.18,
                0.10,
                0.115,
                0.524,
                0.0,
                0.0,
                publish_sonar_tf,
            ),
            static_tf_node(
                "base_link_to_sonar1",
                base_frame,
                "sonar1",
                0.20,
                0.0,
                0.115,
                0.0,
                0.0,
                0.0,
                publish_sonar_tf,
            ),
            static_tf_node(
                "base_link_to_sonar2",
                base_frame,
                "sonar2",
                0.18,
                -0.10,
                0.115,
                -0.524,
                0.0,
                0.0,
                publish_sonar_tf,
            ),
            static_tf_node(
                "base_link_to_sonar3",
                base_frame,
                "sonar3",
                -0.20,
                0.0,
                0.115,
                3.14,
                0.0,
                0.0,
                publish_sonar_tf,
            ),
        ]
    )
