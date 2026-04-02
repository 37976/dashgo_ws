from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from dashgo_web_control.show_web_qr import print_web_qr


def print_web_qr_action(context):
    print_web_qr(
        LaunchConfiguration("host").perform(context),
        LaunchConfiguration("port").perform(context),
    )
    return []


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
            TimerAction(
                period=1.5,
                actions=[OpaqueFunction(function=print_web_qr_action)],
            ),
        ]
    )
