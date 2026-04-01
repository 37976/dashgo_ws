from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8080"),
            DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
            Node(
                package="dashgo_web_control",
                executable="web_control_node",
                name="dashgo_web_control",
                output="screen",
                parameters=[
                    {
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "image_topic": LaunchConfiguration("image_topic"),
                    }
                ],
            ),
        ]
    )
